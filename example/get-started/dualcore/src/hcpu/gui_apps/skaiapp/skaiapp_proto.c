/**
 * @file   skaiapp_proto.c
 * @brief  SkaiApp BLE install path: chunk reassembly (KEY 0x13), remove (0x14),
 *         ack (0x15) — plus MSH commands for sim / console-driven testing.
 *
 * Runs on the BLE parse thread (same as the other communicate_parse_* handlers;
 * health parse writes flash there too, so a package install is in-precedent).
 * The reassembly buffer exists only between seq=0 and the last chunk — nothing
 * static (SRAM budget). Every field from the wire is bounds-checked.
 *
 * Wire (ADR-0037 D3, lockstep with WatchProtocol.kt):
 *   0x13 skaiappPush   {"id","seq","n","crc"(seq0),"chunk"(base64 ≤3KB raw)}
 *   0x14 skaiappRemove {"id"}
 *   0x15 skaiappAck    {"id","code"} ← commu_send_skaiapp_ack()
 *
 * The same push also carries SIGNED JS apps (ADR-0019 Phase 3). The reassembled
 * bytes are then not JSON but a small binary envelope, told apart by its magic:
 *
 *   "SKJS" | flags u8 | 0 u8 | 0 u16 | manifest_len u32 LE | manifest | payload
 *
 * flags bit0 = open the app once it is installed. The manifest and payload go
 * to skai_pkg_install() untouched — the signature covers those exact bytes —
 * and the outcome is reported on 0x29 skaiappRun ("install", result name) as
 * well as the plain 0x15 ack, because "code 2" is not something the AI that
 * wrote the app can act on and "signature" or "limit" is.
 */
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "cJSON.h"
#include "skaiapp_pkg.h"
#include "skaiapp_store.h"
#include "communicate_task.h"
#include "mem_section.h"
#ifdef PKG_USING_QUICKJS
#include "skai/skai_pkg.h"
#include "gui_app_fwk.h"
#include "ui_handler.h"
#endif

#define DBG_TAG "skaiapp.proto"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define CHUNK_RAW_MAX 3072
/* A declarative package is <= 8 KB, but a JS app (source + manifest) is up to
   ~50 KB, so the reassembly buffer is sized for the larger and lives in PSRAM:
   it used to be an 8 KB rt_malloc from HCPU SRAM for the length of a transfer,
   and 60 KB there is not on offer. One transfer at a time, same as before. */
#define CHUNKS_MAX    20
#define REASM_CAP     (CHUNKS_MAX * CHUNK_RAW_MAX)

L2_RET_BSS_SECT_BEGIN(skaiapp_reasm)
static uint8_t s_reasm_buf[REASM_CAP] L2_RET_BSS_SECT(skaiapp_reasm);
L2_RET_BSS_SECT_END

static struct
{
    bool     active;
    char     id[SKAIAPP_ID_MAX];
    uint32_t crc;
    int      n_chunks;
    int      next_seq;
    uint8_t *buf;
    uint32_t used;
} s_re;

static void session_reset(void)
{
    memset(&s_re, 0, sizeof(s_re));
}

#ifdef PKG_USING_QUICKJS
#define SKJS_HDR 12

/* Map a package result onto the v0 ack codes the phone already understands. */
static int pkg_ack_code(skai_pkg_result_t r)
{
    switch (r)
    {
    case SKAI_PKG_OK:          return SKAIAPP_ACK_OK;
    case SKAI_PKG_ERR_VERSION: return SKAIAPP_ACK_UNSUPPORTED;
    case SKAI_PKG_ERR_STORAGE: return SKAIAPP_ACK_STORAGE;
    case SKAI_PKG_ERR_LIMIT:   return SKAIAPP_ACK_LIMIT;
    case SKAI_PKG_ERR_DIGEST:  return SKAIAPP_ACK_CRC;
    default:                   return SKAIAPP_ACK_PARSE;
    }
}

/* A reassembled "SKJS" envelope: install the signed package, report, and open
   it if asked. Runs on the BLE parse thread; launching only posts to the GUI
   thread (gui_app_run), so nothing here touches LVGL. */
static int install_js(const char *wire_id, const uint8_t *buf, uint32_t len)
{
    static skai_pkg_info_t info;   /* ~1.4 KB: not on the BLE thread's stack */
    uint32_t mlen;
    uint8_t flags;
    skai_pkg_result_t r;

    if (len < SKJS_HDR)
        return SKAIAPP_ACK_PARSE;
    flags = buf[4];
    mlen = (uint32_t)buf[8] | ((uint32_t)buf[9] << 8) | ((uint32_t)buf[10] << 16) |
           ((uint32_t)buf[11] << 24);
    if (mlen == 0 || mlen > len - SKJS_HDR)
    {
        commu_send_skaiapp_run(wire_id, "install", "parse", "bad envelope");
        return SKAIAPP_ACK_PARSE;
    }

    r = skai_pkg_install((const char *)buf + SKJS_HDR, mlen,
                         buf + SKJS_HDR + mlen, len - SKJS_HDR - mlen, &info);
    commu_send_skaiapp_run(wire_id, "install", skai_pkg_result_name(r), NULL);
    if (r != SKAI_PKG_OK)
    {
        LOG_W("js install %s: %s", wire_id, skai_pkg_result_name(r));
        return pkg_ack_code(r);
    }
    if (strcmp(info.app_id, wire_id) != 0)
    {
        /* Installed under the manifest's id, which is what the launcher and
           every later report use; say so rather than let the phone wait on the
           wire id forever. */
        LOG_W("js install: wire id %s, manifest id %s", wire_id, info.app_id);
    }

    if (flags & 0x01)
    {
        /* An older build of this app may be on screen: close it first so the
           run below starts the new code instead of resuming the old. Both are
           queued to the GUI task in this order. */
        gui_app_exit(APP_ID_SKAIJS);
        r = skai_pkg_launch(info.keyid, info.app_id);
        if (r != SKAI_PKG_OK)
            commu_send_skaiapp_run(info.app_id, "start", skai_pkg_result_name(r), NULL);
    }
    return SKAIAPP_ACK_OK;
}

/* A signed install must NOT run on the BLE parse thread. It verifies an ECDSA
   signature, writes the package to NAND and — the first time after a boot —
   opens the blocklist KVDB, whose fdb_kvdb_init scans the prefdb region for
   seconds. With the BLE host thread blocked that long the LCPU gives up
   ("LCPU Crash triggered") and the watch resets: that is exactly what the first
   push from the phone after every reboot did on the bench watch. So the
   reassembled envelope is handed to a short-lived worker and the parse thread
   returns at once; the worker sends the ack. The reassembly buffer stays owned
   by the worker until it finishes — a new push meanwhile is refused as busy. */
static volatile bool s_js_busy;
static char          s_js_wire_id[SKAIAPP_ID_MAX];
static uint32_t      s_js_len;

static void js_install_worker(void *arg)
{
    (void)arg;
    int ack = install_js(s_js_wire_id, s_reasm_buf, s_js_len);
    commu_send_skaiapp_ack(s_js_wire_id, ack);
    s_js_busy = false;
}

/* The two KVDB stores a JS app touches — the publisher blocklist (every
   install/launch) and skai.persist (the app's own saved values) — each cost a
   multi-second flash scan the FIRST time after a boot. Pay that once, early, on
   a low-priority thread, rather than inside an install or on the GUI thread the
   first time an app saves something. */
static void js_prefs_warm(void *arg)
{
    (void)arg;
    rt_thread_mdelay(20000);   /* after boot's own flash traffic */
    (void)skai_pkg_blocked("AAAAAAAAAAA");
    extern int32_t skai_persist_get_int(const char *key, int32_t fallback);
    (void)skai_persist_get_int("warm", 0);
}

static int js_prefs_warm_init(void)
{
    rt_thread_t t = rt_thread_create("skjsw", js_prefs_warm, RT_NULL, 4096, 29, 10);
    if (t != RT_NULL)
        rt_thread_startup(t);
    return 0;
}
INIT_APP_EXPORT(js_prefs_warm_init);

/* Returns the ack to send now, or -1 when the worker will send it. */
static int install_js_async(const char *wire_id, uint32_t len)
{
    rt_thread_t t;

    s_js_busy = true;
    strncpy(s_js_wire_id, wire_id, sizeof(s_js_wire_id) - 1);
    s_js_wire_id[sizeof(s_js_wire_id) - 1] = '\0';
    s_js_len = len;
    t = rt_thread_create("skjs", js_install_worker, RT_NULL, 8192, 22, 10);
    if (t == RT_NULL)
    {
        s_js_busy = false;
        commu_send_skaiapp_run(wire_id, "install", "storage", "no memory for the installer");
        return SKAIAPP_ACK_STORAGE;
    }
    rt_thread_startup(t);
    return -1;
}
#endif /* PKG_USING_QUICKJS */

static bool wire_id_ok(const char *id)
{
    size_t n = (id != NULL) ? strlen(id) : 0;
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

void skaiapp_on_push_chunk(const uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        return;
    }
    cJSON *root = cJSON_ParseWithLength((const char *)pValue, length);
    if (root == NULL)
    {
        LOG_W("push: bad json frame");
        return; /* no id to ack against */
    }

    char id[SKAIAPP_ID_MAX] = "";
    const cJSON *jid = cJSON_GetObjectItem(root, "id");
    if (jid != NULL && cJSON_IsString(jid) && jid->valuestring != NULL)
    {
        strncpy(id, jid->valuestring, sizeof(id) - 1);
    }
    const cJSON *jseq = cJSON_GetObjectItem(root, "seq");
    const cJSON *jn = cJSON_GetObjectItem(root, "n");
    const cJSON *jchunk = cJSON_GetObjectItem(root, "chunk");
    int seq = (jseq != NULL && cJSON_IsNumber(jseq)) ? (int)jseq->valuedouble : -1;
    int n = (jn != NULL && cJSON_IsNumber(jn)) ? (int)jn->valuedouble : -1;
    const char *chunk = (jchunk != NULL && cJSON_IsString(jchunk))
                        ? jchunk->valuestring : NULL;

    int ack = -1; /* <0 = no ack yet (mid-transfer) */

    if (!wire_id_ok(id) || seq < 0 || n < 1 || n > CHUNKS_MAX || chunk == NULL)
    {
        ack = SKAIAPP_ACK_PARSE;
        session_reset();
        goto out;
    }

    if (seq == 0)
    {
#ifdef PKG_USING_QUICKJS
        if (s_js_busy)
        {
            /* The previous JS install still owns the reassembly buffer. */
            ack = SKAIAPP_ACK_STORAGE;
            goto out;
        }
#endif
        session_reset();
        const cJSON *jcrc = cJSON_GetObjectItem(root, "crc");
        if (jcrc == NULL || !cJSON_IsNumber(jcrc))
        {
            ack = SKAIAPP_ACK_PARSE;
            goto out;
        }
        s_re.buf = s_reasm_buf;
        s_re.active = true;
        strncpy(s_re.id, id, sizeof(s_re.id) - 1);
        /* JSON numbers are doubles — u32 crc survives exactly (≤2^53) */
        s_re.crc = (uint32_t)jcrc->valuedouble;
        s_re.n_chunks = n;
        s_re.next_seq = 0;
    }

    if (!s_re.active || strcmp(s_re.id, id) != 0 || seq != s_re.next_seq ||
        n != s_re.n_chunks)
    {
        LOG_W("push: out-of-order (id=%s seq=%d expect=%d)", id, seq, s_re.next_seq);
        ack = SKAIAPP_ACK_PARSE;
        session_reset();
        goto out;
    }

    {
        int dec = skaiapp_b64_decode(chunk, s_re.buf + s_re.used,
                                     REASM_CAP - s_re.used);
        if (dec < 0 || dec > CHUNK_RAW_MAX)
        {
            ack = SKAIAPP_ACK_PARSE;
            session_reset();
            goto out;
        }
        s_re.used += (uint32_t)dec;
        s_re.next_seq++;
    }

    if (s_re.next_seq == s_re.n_chunks) /* complete */
    {
        if (skaiapp_crc32(s_re.buf, s_re.used) != s_re.crc)
        {
            LOG_W("push: crc mismatch (%u bytes)", s_re.used);
            ack = SKAIAPP_ACK_CRC;
        }
#ifdef PKG_USING_QUICKJS
        else if (s_re.used >= 4 && memcmp(s_re.buf, "SKJS", 4) == 0)
        {
            ack = install_js_async(id, s_re.used);
        }
#endif
        else if (s_re.used > SKAIAPP_PKG_MAX_BYTES)
        {
            ack = SKAIAPP_ACK_LIMIT; /* declarative packages keep their 8 KB cap */
        }
        else
        {
            ack = skaiapp_store_install(s_re.buf, s_re.used, NULL);
        }
        session_reset();
    }

out:
    cJSON_Delete(root);
    if (ack >= 0 && id[0] != '\0')
    {
        commu_send_skaiapp_ack(id, ack);
    }
}

void skaiapp_on_remove(const uint8_t *pValue, uint16_t length)
{
    if (pValue == NULL || length == 0)
    {
        return;
    }
    cJSON *root = cJSON_ParseWithLength((const char *)pValue, length);
    if (root == NULL)
    {
        return;
    }
    char id[SKAIAPP_ID_MAX] = "";
    char keyid[16] = "";
    const cJSON *jid = cJSON_GetObjectItem(root, "id");
    if (jid != NULL && cJSON_IsString(jid) && jid->valuestring != NULL)
    {
        strncpy(id, jid->valuestring, sizeof(id) - 1);
    }
    /* "k" = publisher keyid: present only for a signed JS app, whose storage
       key is (keyid, id). */
    const cJSON *jk = cJSON_GetObjectItem(root, "k");
    if (jk != NULL && cJSON_IsString(jk) && jk->valuestring != NULL)
    {
        strncpy(keyid, jk->valuestring, sizeof(keyid) - 1);
    }
    cJSON_Delete(root);

    if (!wire_id_ok(id))
    {
        return;
    }
#ifdef PKG_USING_QUICKJS
    if (keyid[0] != '\0')
    {
        skai_pkg_result_t r = skai_pkg_remove(keyid, id);
        commu_send_skaiapp_ack(id, r == SKAI_PKG_OK ? SKAIAPP_ACK_OK : SKAIAPP_ACK_PARSE);
        return;
    }
#endif
    int code = skaiapp_store_remove(id);
    commu_send_skaiapp_ack(id, code);
}

/* ── console / sim test commands ── */
#ifdef FINSH_USING_MSH
#include <finsh.h>
#include <dfs_posix.h>

/* skaiapp_install <fs-path.json> — run the exact install path from a file */
static void skaiapp_install(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: skaiapp_install <path.json>\n");
        return;
    }
    int fd = open(argv[1], O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        rt_kprintf("open %s failed\n", argv[1]);
        return;
    }
    uint8_t *buf = rt_malloc(SKAIAPP_PKG_MAX_BYTES + 1);
    if (buf == NULL)
    {
        close(fd);
        return;
    }
    int n = read(fd, buf, SKAIAPP_PKG_MAX_BYTES + 1);
    close(fd);
    if (n <= 0 || n > SKAIAPP_PKG_MAX_BYTES)
    {
        rt_kprintf("read failed / too big (%d)\n", n);
        rt_free(buf);
        return;
    }
    int code = skaiapp_store_install(buf, (uint32_t)n, NULL);
    rt_kprintf("install ack=%d (0=ok)\n", code);
    rt_free(buf);
}
MSH_CMD_EXPORT(skaiapp_install, install a skaiapp package from a json file);

static void skaiapp_rm(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: skaiapp_rm <id>\n");
        return;
    }
    rt_kprintf("remove ack=%d\n", skaiapp_store_remove(argv[1]));
}
MSH_CMD_EXPORT(skaiapp_rm, remove an installed skaiapp by id);

static void skaiapp_ls(int argc, char **argv)
{
    (void)argc; (void)argv;
    char id[SKAIAPP_ID_MAX], name[SKAIAPP_NAME_MAX];
    uint8_t icon;
    int n = skaiapp_store_count();
    rt_kprintf("%d skaiapp(s):\n", n);
    for (int i = 0; i < n; i++)
    {
        if (skaiapp_store_meta(i, id, name, &icon))
        {
            rt_kprintf("  [%d] %s (%s) icon=%d\n", i, id, name, icon);
        }
    }
}
MSH_CMD_EXPORT(skaiapp_ls, list installed skaiapps);

#ifdef PKG_USING_QUICKJS
/* skaijs_install <file> — feed a phone-built "SKJS" envelope through the exact
   install path the BLE push takes (verify signature, install, open). This is
   how a package the PHONE signed is checked against the WATCH's verifier
   without a radio in between. */
static void skaijs_install(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("usage: skaijs_install <file.skjs>\n");
        return;
    }
    /* skai_fopen/skai_fread: plain open() binds to the host CRT on the
       simulator and never sees the simulated filesystem (skai_pkg.h). */
    int fd = skai_fopen(argv[1], O_RDONLY | O_BINARY);
    if (fd < 0)
    {
        rt_kprintf("open %s failed\n", argv[1]);
        return;
    }
    int n = skai_fread(fd, s_reasm_buf, REASM_CAP);
    skai_fclose(fd);
    if (n <= 0)
    {
        rt_kprintf("read failed (%d)\n", n);
        return;
    }
    rt_kprintf("skaijs_install: %d bytes -> ack %d (0=ok)\n", n,
               install_js("file", s_reasm_buf, (uint32_t)n));
}
MSH_CMD_EXPORT(skaijs_install, install and open a signed JS app envelope from a file);
#endif

/* b64w <path> <w|a> <base64> — write (w) or append (a) base64-decoded bytes to a
   file. The console line is 80 characters, so a file arrives as many short
   appends; this is how a test package reaches a bench watch that has no phone
   paired to it. Dev console only. */
static void b64w(int argc, char **argv)
{
    uint8_t out[64];
    if (argc < 4 || (argv[2][0] != 'w' && argv[2][0] != 'a'))
    {
        rt_kprintf("usage: b64w <path> <w|a> <base64>\n");
        return;
    }
    int n = skaiapp_b64_decode(argv[3], out, sizeof(out));
    if (n < 0)
    {
        rt_kprintf("b64w: bad base64\n");
        return;
    }
    int flags = O_WRONLY | O_CREAT | O_BINARY | (argv[2][0] == 'a' ? O_APPEND : O_TRUNC);
#ifdef PKG_USING_QUICKJS
    int fd = skai_fopen(argv[1], flags);
#else
    int fd = open(argv[1], flags);
#endif
    if (fd < 0)
    {
        rt_kprintf("b64w: open %s failed\n", argv[1]);
        return;
    }
#ifdef PKG_USING_QUICKJS
    int w = skai_fwrite(fd, out, n);
    skai_fclose(fd);
#else
    int w = write(fd, out, n);
    close(fd);
#endif
    rt_kprintf("b64w %d\n", w);
}
MSH_CMD_EXPORT(b64w, append base64 bytes to a file (dev upload));
#endif /* FINSH_USING_MSH */
