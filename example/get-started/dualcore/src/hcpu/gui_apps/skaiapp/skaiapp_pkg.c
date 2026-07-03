/**
 * @file   skaiapp_pkg.c
 * @brief  SkaiApp package parser (JSON → bounded render model + engine seed).
 *
 * TOLERANT parser over EXTERNAL input: every string copy is bounded, unknown
 * fields/widgets are skipped, counts are clamped to the schema caps, and the
 * package id is charset-whitelisted because it becomes a filename under
 * /skaiapp/ (no traversal). The phone-side validator is the strict layer.
 */
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "cJSON.h"
#include "skaiapp_pkg.h"
#include "skaiapp_engine.h"

#define DBG_TAG "skaiapp.pkg"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

/* ── CRC-32 (IEEE reflected, bitwise — packages are ≤8 KB, no table needed;
      parameters match java.util.zip.CRC32 on the phone side) ── */
uint32_t skaiapp_crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

/* ── base64 decode (standard alphabet, '=' padding, no whitespace) ── */
static int8_t b64_val(char c)
{
    if (c >= 'A' && c <= 'Z') return (int8_t)(c - 'A');
    if (c >= 'a' && c <= 'z') return (int8_t)(c - 'a' + 26);
    if (c >= '0' && c <= '9') return (int8_t)(c - '0' + 52);
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

int skaiapp_b64_decode(const char *src, uint8_t *dst, uint32_t dst_cap)
{
    uint32_t out = 0, acc = 0;
    int nbits = 0;
    for (const char *p = src; *p != '\0'; p++)
    {
        if (*p == '=')
        {
            break;
        }
        int8_t v = b64_val(*p);
        if (v < 0)
        {
            return -1;
        }
        acc = (acc << 6) | (uint32_t)v;
        nbits += 6;
        if (nbits >= 8)
        {
            nbits -= 8;
            if (out >= dst_cap)
            {
                return -1;
            }
            dst[out++] = (uint8_t)((acc >> nbits) & 0xFF);
        }
    }
    return (int)out;
}

/* ── small helpers ── */

const char *skaiapp_model_text(const skaiapp_model_t *m, uint16_t off)
{
    if (m == NULL || off == 0xFFFF || off >= m->strpool_used)
    {
        return "";
    }
    return &m->strpool[off];
}

/* bounded copy of a cJSON string into a fixed array; "" when absent */
static void copy_str(char *dst, size_t cap, const cJSON *j)
{
    if (j != NULL && cJSON_IsString(j) && j->valuestring != NULL)
    {
        strncpy(dst, j->valuestring, cap - 1);
        dst[cap - 1] = '\0';
    }
    else
    {
        dst[0] = '\0';
    }
}

/* intern a cJSON string into the model pool; 0xFFFF when absent/full */
static uint16_t pool_add(skaiapp_model_t *m, const cJSON *j)
{
    if (j == NULL || !cJSON_IsString(j) || j->valuestring == NULL)
    {
        return 0xFFFF;
    }
    size_t len = strlen(j->valuestring);
    if (len > SKAIAPP_POOL_STR_MAX) /* memo text is the long case (200 CJK) */
    {
        len = SKAIAPP_POOL_STR_MAX;
    }
    if ((uint32_t)m->strpool_used + len + 1 > SKAIAPP_STRPOOL_MAX)
    {
        LOG_W("strpool full, text dropped");
        return 0xFFFF;
    }
    uint16_t off = m->strpool_used;
    memcpy(&m->strpool[off], j->valuestring, len);
    m->strpool[off + len] = '\0';
    m->strpool_used = (uint16_t)(off + len + 1);
    return off;
}

/* filename-safe id: ^[a-z0-9][a-z0-9-]{0,23}$ (blocks path traversal) */
static bool id_ok(const char *id)
{
    size_t n = strlen(id);
    if (n == 0 || n >= SKAIAPP_ID_MAX)
    {
        return false;
    }
    for (size_t i = 0; i < n; i++)
    {
        char c = id[i];
        bool alnum = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        if (!(alnum || (i > 0 && c == '-')))
        {
            return false;
        }
    }
    return true;
}

static uint8_t map_enum(const cJSON *j, const char *const tbl[], uint8_t n,
                        uint8_t dflt)
{
    if (j == NULL || !cJSON_IsString(j) || j->valuestring == NULL)
    {
        return dflt;
    }
    for (uint8_t i = 0; i < n; i++)
    {
        if (strcmp(j->valuestring, tbl[i]) == 0)
        {
            return i;
        }
    }
    return dflt;
}

/* schema icon enum order — skaiapp_render.c's asset table mirrors this order */
static const char *const k_icons[] =
{
    "water", "timer", "heart", "steps", "battery", "bell",
    "star", "sun", "moon", "coffee", "pill", "run",
};
#define ICON_UNKNOWN 0xFF

static const char *const k_colors[] =
{
    "", "white", "gray", "accent", "red", "orange", "yellow",
    "green", "blue", "purple",
};

static const char *const k_sizes[] = { "s", "m", "l", "xl" };

/* "HH:MM" → minutes-of-day; 0xFFFF invalid */
static uint16_t hhmm_min(const cJSON *j)
{
    if (j == NULL || !cJSON_IsString(j) || j->valuestring == NULL)
    {
        return 0xFFFF;
    }
    const char *s = j->valuestring;
    if (strlen(s) != 5 || s[2] != ':')
    {
        return 0xFFFF;
    }
    int h = (s[0] - '0') * 10 + (s[1] - '0');
    int mi = (s[3] - '0') * 10 + (s[4] - '0');
    if (h < 0 || h > 23 || mi < 0 || mi > 59)
    {
        return 0xFFFF;
    }
    return (uint16_t)(h * 60 + mi);
}

/* find "timer:<id>" / "reminder:<id>" target among collected ids */
static int8_t resolve_ref(const char *ref, const char *prefix,
                          char ids[][SKAIAPP_ID_MAX], uint8_t n)
{
    size_t plen = strlen(prefix);
    if (strncmp(ref, prefix, plen) != 0)
    {
        return -1;
    }
    for (uint8_t i = 0; i < n; i++)
    {
        if (strcmp(ref + plen, ids[i]) == 0)
        {
            return (int8_t)i;
        }
    }
    return -1;
}

/* ── item parsing ── */

typedef struct
{
    char t_ids[SKAIAPP_MAX_TIMERS][SKAIAPP_ID_MAX];
    char r_ids[SKAIAPP_MAX_REMINDERS][SKAIAPP_ID_MAX];
    char m_ids[SKAIAPP_MAX_MEMOS][SKAIAPP_ID_MAX];
    uint8_t nt, nr, nm;
} ref_ctx_t;

static bool parse_leaf(skaiapp_model_t *m, const cJSON *jit, ref_ctx_t *refs)
{
    if (m->n_items >= SKAIAPP_MAX_ITEMS)
    {
        return false;
    }
    const cJSON *jw = cJSON_GetObjectItem(jit, "w");
    if (jw == NULL || !cJSON_IsString(jw) || jw->valuestring == NULL)
    {
        return true; /* tolerant: skip malformed item */
    }
    skaiapp_witem_t *it = &m->items[m->n_items];
    memset(it, 0, sizeof(*it));
    it->bind_idx = -1;
    it->action_idx = -1;
    it->text_off = 0xFFFF;
    const char *w = jw->valuestring;

    if (strcmp(w, "label") == 0)
    {
        it->wtype = SKAIAPP_W_LABEL;
        it->text_off = pool_add(m, cJSON_GetObjectItem(jit, "text"));
        it->size = map_enum(cJSON_GetObjectItem(jit, "size"), k_sizes, 4, 1);
        it->color = map_enum(cJSON_GetObjectItem(jit, "color"), k_colors, 10, 0);
    }
    else if (strcmp(w, "value") == 0)
    {
        it->wtype = SKAIAPP_W_VALUE;
        it->size = map_enum(cJSON_GetObjectItem(jit, "size"), k_sizes, 4, 1);
        it->color = map_enum(cJSON_GetObjectItem(jit, "color"), k_colors, 10, 0);
        const cJSON *jb = cJSON_GetObjectItem(jit, "bind");
        if (jb != NULL && cJSON_IsString(jb) && jb->valuestring != NULL)
        {
            const char *b = jb->valuestring;
            if (strcmp(b, "time") == 0) it->bind = SKAIAPP_BIND_TIME;
            else if (strcmp(b, "date") == 0) it->bind = SKAIAPP_BIND_DATE;
            else if (strcmp(b, "battery") == 0) it->bind = SKAIAPP_BIND_BATTERY;
            else if (strcmp(b, "hr") == 0) it->bind = SKAIAPP_BIND_HR;
            else if (strcmp(b, "steps") == 0) it->bind = SKAIAPP_BIND_STEPS;
            else if ((it->bind_idx = resolve_ref(b, "timer:", refs->t_ids, refs->nt)) >= 0)
                it->bind = SKAIAPP_BIND_TIMER;
            else if ((it->bind_idx = resolve_ref(b, "reminder:", refs->r_ids, refs->nr)) >= 0)
                it->bind = SKAIAPP_BIND_REMINDER;
            else if ((it->bind_idx = resolve_ref(b, "memo:", refs->m_ids, refs->nm)) >= 0)
                it->bind = SKAIAPP_BIND_MEMO;
            else
                LOG_W("value bind unresolved: %s", b);
        }
    }
    else if (strcmp(w, "icon") == 0)
    {
        it->wtype = SKAIAPP_W_ICON;
        it->icon = map_enum(cJSON_GetObjectItem(jit, "name"), k_icons, 12, ICON_UNKNOWN);
        it->size = map_enum(cJSON_GetObjectItem(jit, "size"), k_sizes, 3, 1);
    }
    else if (strcmp(w, "arc") == 0 || strcmp(w, "bar") == 0)
    {
        it->wtype = (w[0] == 'a') ? SKAIAPP_W_ARC : SKAIAPP_W_BAR;
        it->color = map_enum(cJSON_GetObjectItem(jit, "color"), k_colors, 10, 0);
        const cJSON *js = cJSON_GetObjectItem(jit, "size");
        it->size = (js != NULL && cJSON_IsString(js) && js->valuestring != NULL &&
                    strcmp(js->valuestring, "s") == 0) ? 0 : 1;
        const cJSON *jm = cJSON_GetObjectItem(jit, "max");
        if (jm != NULL && cJSON_IsNumber(jm) && jm->valuedouble >= 1 &&
            jm->valuedouble <= 200000)
        {
            it->max = (int32_t)jm->valuedouble;
        }
        const cJSON *jb = cJSON_GetObjectItem(jit, "bind");
        if (jb != NULL && cJSON_IsString(jb) && jb->valuestring != NULL)
        {
            const char *b = jb->valuestring;
            if (strcmp(b, "battery") == 0) it->bind = SKAIAPP_BIND_BATTERY;
            else if (strcmp(b, "steps") == 0) it->bind = SKAIAPP_BIND_STEPS;
            else if ((it->bind_idx = resolve_ref(b, "timer:", refs->t_ids, refs->nt)) >= 0)
                it->bind = SKAIAPP_BIND_TIMER;
            else
                LOG_W("gauge bind unresolved: %s", b);
        }
    }
    else if (strcmp(w, "button") == 0)
    {
        it->wtype = SKAIAPP_W_BUTTON;
        it->text_off = pool_add(m, cJSON_GetObjectItem(jit, "text"));
        const cJSON *jst = cJSON_GetObjectItem(jit, "style");
        it->ghost = (jst != NULL && cJSON_IsString(jst) && jst->valuestring != NULL &&
                     strcmp(jst->valuestring, "ghost") == 0) ? 1 : 0;
        const cJSON *ja = cJSON_GetObjectItem(jit, "action");
        if (ja != NULL && cJSON_IsString(ja) && ja->valuestring != NULL)
        {
            const char *a = ja->valuestring;
            if ((it->action_idx = resolve_ref(a, "timer.start:", refs->t_ids, refs->nt)) >= 0)
                it->action = SKAIAPP_ACT_TIMER_START;
            else if ((it->action_idx = resolve_ref(a, "timer.pause:", refs->t_ids, refs->nt)) >= 0)
                it->action = SKAIAPP_ACT_TIMER_PAUSE;
            else if ((it->action_idx = resolve_ref(a, "timer.reset:", refs->t_ids, refs->nt)) >= 0)
                it->action = SKAIAPP_ACT_TIMER_RESET;
            else if ((it->action_idx = resolve_ref(a, "reminder.toggle:", refs->r_ids, refs->nr)) >= 0)
                it->action = SKAIAPP_ACT_REMINDER_TOGGLE;
            else
                LOG_W("button action unresolved: %s", a); /* incl. v2 phone:* */
        }
    }
    else if (strcmp(w, "spacer") == 0)
    {
        it->wtype = SKAIAPP_W_SPACER;
        const cJSON *jh = cJSON_GetObjectItem(jit, "h");
        int h = (jh != NULL && cJSON_IsNumber(jh)) ? (int)jh->valuedouble : 16;
        it->spacer_h = (h == 8 || h == 16 || h == 24) ? (uint8_t)h : 16;
    }
    else
    {
        LOG_W("unknown widget '%s' skipped", w);
        return true; /* forward compat */
    }
    m->n_items++;
    return true;
}

/* ── timers / reminders (two-pass: ids first, then details) ── */

static void collect_ids(const cJSON *arr, char ids[][SKAIAPP_ID_MAX],
                        uint8_t max, uint8_t *n_out)
{
    *n_out = 0;
    if (arr == NULL || !cJSON_IsArray(arr))
    {
        return;
    }
    const cJSON *e = NULL;
    cJSON_ArrayForEach(e, arr)
    {
        if (*n_out >= max)
        {
            break;
        }
        char tmp[SKAIAPP_ID_MAX];
        copy_str(tmp, sizeof(tmp), cJSON_GetObjectItem(e, "id"));
        if (!id_ok(tmp))
        {
            tmp[0] = '\0';
        }
        strncpy(ids[*n_out], tmp, SKAIAPP_ID_MAX - 1);
        ids[*n_out][SKAIAPP_ID_MAX - 1] = '\0';
        (*n_out)++;
    }
}

static void parse_timers(const cJSON *arr, skaiapp_model_t *m,
                         skaiapp_eng_seed_t *seed, ref_ctx_t *refs)
{
    if (arr == NULL || !cJSON_IsArray(arr))
    {
        return;
    }
    uint8_t i = 0;
    const cJSON *e = NULL;
    cJSON_ArrayForEach(e, arr)
    {
        if (i >= SKAIAPP_MAX_TIMERS)
        {
            break;
        }
        skaiapp_seed_timer_t *t = &seed->t[i];
        const cJSON *jd = cJSON_GetObjectItem(e, "duration_s");
        int32_t dur = (jd != NULL && cJSON_IsNumber(jd)) ? (int32_t)jd->valuedouble : 60;
        if (dur < 5) dur = 5;
        if (dur > 86400) dur = 86400;
        t->duration_s = (uint32_t)dur;
        t->next = -1;
        t->flags = 0;
        copy_str(m->timer_label[i], SKAIAPP_NAME_MAX, cJSON_GetObjectItem(e, "label"));
        if (cJSON_IsTrue(cJSON_GetObjectItem(e, "autostart")))
        {
            t->flags |= SKAIAPP_TF_AUTOSTART;
        }
        const cJSON *jf = cJSON_GetObjectItem(e, "on_fire");
        if (jf != NULL && cJSON_IsObject(jf))
        {
            const cJSON *jn = cJSON_GetObjectItem(jf, "notify");
            if (jn != NULL && cJSON_IsString(jn) && jn->valuestring != NULL &&
                jn->valuestring[0] != '\0')
            {
                t->flags |= SKAIAPP_TF_HAS_NOTIFY;
            }
            if (cJSON_IsTrue(cJSON_GetObjectItem(jf, "vibrate")))
            {
                t->flags |= SKAIAPP_TF_VIBRATE;
            }
            if (cJSON_IsTrue(cJSON_GetObjectItem(jf, "autostart_next")))
            {
                t->flags |= SKAIAPP_TF_AUTOSTART_NEXT;
            }
            const cJSON *jx = cJSON_GetObjectItem(jf, "next");
            if (jx != NULL && cJSON_IsString(jx) && jx->valuestring != NULL)
            {
                for (uint8_t k = 0; k < refs->nt; k++)
                {
                    if (strcmp(jx->valuestring, refs->t_ids[k]) == 0)
                    {
                        t->next = (int8_t)k;
                        break;
                    }
                }
            }
        }
        i++;
    }
    seed->n_timers = i;
    m->n_timers = i;
}

static void parse_reminders(const cJSON *arr, skaiapp_model_t *m,
                            skaiapp_eng_seed_t *seed)
{
    if (arr == NULL || !cJSON_IsArray(arr))
    {
        return;
    }
    uint8_t i = 0;
    const cJSON *e = NULL;
    cJSON_ArrayForEach(e, arr)
    {
        if (i >= SKAIAPP_MAX_REMINDERS)
        {
            break;
        }
        skaiapp_seed_rem_t *r = &seed->r[i];
        memset(r, 0, sizeof(*r));
        const cJSON *jk = cJSON_GetObjectItem(e, "kind");
        bool daily = (jk != NULL && cJSON_IsString(jk) && jk->valuestring != NULL &&
                      strcmp(jk->valuestring, "daily") == 0);
        r->kind = daily ? 1 : 0;
        const cJSON *jen = cJSON_GetObjectItem(e, "enabled");
        r->enabled = (jen == NULL) ? 1 : (cJSON_IsTrue(jen) ? 1 : 0);
        const cJSON *jv = cJSON_GetObjectItem(e, "vibrate");
        r->vibrate = (jv == NULL) ? 1 : (cJSON_IsTrue(jv) ? 1 : 0);
        if (daily)
        {
            const cJSON *jt = cJSON_GetObjectItem(e, "times");
            if (jt != NULL && cJSON_IsArray(jt))
            {
                const cJSON *tt = NULL;
                cJSON_ArrayForEach(tt, jt)
                {
                    if (r->n_times >= SKAIAPP_MAX_DAILY_TIMES)
                    {
                        break;
                    }
                    uint16_t mm = hhmm_min(tt);
                    if (mm != 0xFFFF)
                    {
                        r->times[r->n_times++] = mm;
                    }
                }
            }
        }
        else
        {
            const cJSON *je = cJSON_GetObjectItem(e, "every_min");
            int32_t ev = (je != NULL && cJSON_IsNumber(je)) ? (int32_t)je->valuedouble : 60;
            if (ev < 5) ev = 5;
            if (ev > 720) ev = 720;
            r->every_min = (uint16_t)ev;
            const cJSON *jw = cJSON_GetObjectItem(e, "window");
            if (jw != NULL && cJSON_IsObject(jw))
            {
                uint16_t ws = hhmm_min(cJSON_GetObjectItem(jw, "start"));
                uint16_t we = hhmm_min(cJSON_GetObjectItem(jw, "end"));
                if (ws != 0xFFFF && we != 0xFFFF)
                {
                    r->win_start = ws;
                    r->win_end = we;
                }
            }
        }
        i++;
    }
    seed->n_reminders = i;
    m->n_reminders = i;
}

/* memos: user-authored text, interned into the model strpool (render-only, no
   engine seed). ids were already collected for bind resolution. */
static void parse_memos(const cJSON *arr, skaiapp_model_t *m)
{
    uint8_t i = 0;
    const cJSON *e = NULL;
    if (arr != NULL && cJSON_IsArray(arr))
    {
        cJSON_ArrayForEach(e, arr)
        {
            if (i >= SKAIAPP_MAX_MEMOS)
            {
                break;
            }
            m->memo_text_off[i] = pool_add(m, cJSON_GetObjectItem(e, "text"));
            copy_str(m->memo_id[i], SKAIAPP_ID_MAX, cJSON_GetObjectItem(e, "id"));
            i++;
        }
    }
    m->n_memos = i;
}

/* ── top level ── */

int skaiapp_pkg_parse(const uint8_t *json, uint32_t len, skaiapp_model_t *m,
                      struct skaiapp_eng_seed *seed_out)
{
    if (json == NULL || len == 0 || len > SKAIAPP_PKG_MAX_BYTES || m == NULL)
    {
        return -3;
    }
    cJSON *root = cJSON_ParseWithLength((const char *)json, len);
    if (root == NULL)
    {
        return -1;
    }
    int ret = 0;
    memset(m, 0, sizeof(*m));
    m->strpool_used = 1; /* offset 0 reserved so 0xFFFF is the only "none" */

    const cJSON *jv = cJSON_GetObjectItem(root, "skaiapp");
    if (jv == NULL || !cJSON_IsNumber(jv) ||
        (int)jv->valuedouble > SKAIAPP_SCHEMA_MAJOR)
    {
        ret = -2;
        goto out;
    }
    copy_str(m->id, sizeof(m->id), cJSON_GetObjectItem(root, "id"));
    if (!id_ok(m->id))
    {
        ret = -3;
        goto out;
    }
    copy_str(m->name, sizeof(m->name), cJSON_GetObjectItem(root, "name"));
    if (m->name[0] == '\0')
    {
        strncpy(m->name, m->id, sizeof(m->name) - 1);
    }
    m->icon = map_enum(cJSON_GetObjectItem(root, "icon"), k_icons, 12, ICON_UNKNOWN);
    m->accent = map_enum(cJSON_GetObjectItem(root, "accent"), k_colors, 10,
                         SKAIAPP_COL_BLUE);

    /* refs first so item binds/actions resolve to slots */
    ref_ctx_t refs;
    memset(&refs, 0, sizeof(refs));
    collect_ids(cJSON_GetObjectItem(root, "timers"), refs.t_ids,
                SKAIAPP_MAX_TIMERS, &refs.nt);
    collect_ids(cJSON_GetObjectItem(root, "reminders"), refs.r_ids,
                SKAIAPP_MAX_REMINDERS, &refs.nr);
    collect_ids(cJSON_GetObjectItem(root, "memos"), refs.m_ids,
                SKAIAPP_MAX_MEMOS, &refs.nm);

    skaiapp_eng_seed_t seed;
    memset(&seed, 0, sizeof(seed));
    parse_timers(cJSON_GetObjectItem(root, "timers"), m, &seed, &refs);
    parse_reminders(cJSON_GetObjectItem(root, "reminders"), m, &seed);
    parse_memos(cJSON_GetObjectItem(root, "memos"), m);

    const cJSON *jp = cJSON_GetObjectItem(root, "page");
    const cJSON *jitems = (jp != NULL) ? cJSON_GetObjectItem(jp, "items") : NULL;
    if (jitems == NULL || !cJSON_IsArray(jitems))
    {
        ret = -3;
        goto out;
    }
    int top = 0;
    const cJSON *jit = NULL;
    cJSON_ArrayForEach(jit, jitems)
    {
        if (++top > SKAIAPP_MAX_TOP_ITEMS)
        {
            break; /* clamp, don't reject */
        }
        const cJSON *jw = cJSON_GetObjectItem(jit, "w");
        bool is_row = (jw != NULL && cJSON_IsString(jw) && jw->valuestring != NULL &&
                       strcmp(jw->valuestring, "row") == 0);
        if (is_row)
        {
            if (m->n_items >= SKAIAPP_MAX_ITEMS)
            {
                break;
            }
            skaiapp_witem_t *row = &m->items[m->n_items++];
            memset(row, 0, sizeof(*row));
            row->wtype = SKAIAPP_W_ROW;
            row->text_off = 0xFFFF;
            row->bind_idx = -1;
            row->action_idx = -1;
            const cJSON *jch = cJSON_GetObjectItem(jit, "items");
            uint8_t emitted = 0;
            if (jch != NULL && cJSON_IsArray(jch))
            {
                const cJSON *jc = NULL;
                cJSON_ArrayForEach(jc, jch)
                {
                    if (emitted >= SKAIAPP_MAX_ROW_CHILD)
                    {
                        break;
                    }
                    const cJSON *jcw = cJSON_GetObjectItem(jc, "w");
                    if (jcw != NULL && cJSON_IsString(jcw) && jcw->valuestring != NULL &&
                        strcmp(jcw->valuestring, "row") == 0)
                    {
                        continue; /* no nested rows */
                    }
                    uint8_t before = m->n_items;
                    if (!parse_leaf(m, jc, &refs))
                    {
                        break;
                    }
                    if (m->n_items > before)
                    {
                        emitted++;
                    }
                }
            }
            row->row_n = emitted;
        }
        else
        {
            if (!parse_leaf(m, jit, &refs))
            {
                break;
            }
        }
    }
    if (m->n_items == 0)
    {
        ret = -3;
        goto out;
    }

    if (seed_out != NULL)
    {
        memcpy(seed_out, &seed, sizeof(seed));
    }

out:
    cJSON_Delete(root);
    return ret;
}
