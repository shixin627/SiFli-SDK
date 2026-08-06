/*
 * Skai SDK — signed app packages (ADR-0019 §2.5). See skai_pkg.h for the shape
 * of a package and why the manifest and the payload are separate blobs.
 *
 * Everything in here treats its input as hostile, because it is: a package
 * arrives from a phone, was built by anyone at all, and the signature grants no
 * authority. The order of the checks below is deliberate — the cheap
 * structural ones first, so a malformed or revoked package never reaches the
 * curve arithmetic.
 */
#include <string.h>
#include <stdlib.h>

#include <rtthread.h>
#include <dfs_posix.h>

#include "cJSON.h"
#include "mbedtls/sha256.h"
#include "mbedtls/ecdsa.h"

#include "share_prefs.h"

#include "skai/skai_js.h"
#include "skai/skai_pkg.h"
#include "skai/skai_sdk_version.h"

#define DBG_TAG "skai.pkg"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define PKG_DIR "/skaiapp"

/* ─────────────────────────────── base64url ──────────────────────────────── */

/* Strict on purpose: no padding, no whitespace, no alternate alphabet. A
 * decoder that accepts sloppy input gives an attacker two spellings of the same
 * keyid, and keyid equality is what TOFU and the blocklist rest on. */
static int b64u_decode(const char *in, uint32_t inlen, uint8_t *out, uint32_t cap)
{
    uint32_t acc = 0, nbits = 0, n = 0;

    for (uint32_t i = 0; i < inlen; i++)
    {
        char c = in[i];
        int v;

        if      (c >= 'A' && c <= 'Z') v = c - 'A';
        else if (c >= 'a' && c <= 'z') v = c - 'a' + 26;
        else if (c >= '0' && c <= '9') v = c - '0' + 52;
        else if (c == '-')             v = 62;
        else if (c == '_')             v = 63;
        else                           return -1;

        acc = (acc << 6) | (uint32_t)v;
        nbits += 6;
        if (nbits >= 8)
        {
            nbits -= 8;
            if (n >= cap)
                return -1;
            out[n++] = (uint8_t)((acc >> nbits) & 0xFF);
        }
    }
    /* Leftover bits must be zero, or two different inputs decode alike. */
    if (nbits >= 6 || (acc & ((1u << nbits) - 1u)) != 0)
        return -1;
    return (int)n;
}

static void sha256(const uint8_t *data, uint32_t len, uint8_t out[32])
{
    mbedtls_sha256_context c;

    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    mbedtls_sha256_update(&c, data, len);
    mbedtls_sha256_finish(&c, out);
    mbedtls_sha256_free(&c);
}

/* ─────────────────────────────── blocklist ──────────────────────────────── */

/* Keyids only, 8 raw bytes each — the schema's `reason` is dropped until there
 * is a UI to show it. Stored in the KVDB rather than a file so a corrupt
 * filesystem cannot quietly un-revoke a publisher. */
#define BL_PREFS   "skai_pkg"
#define BL_SEQ_KEY "bl_seq"
#define BL_KEY     "bl_keys"
#define BL_MAX     128

/* Cached, because skai_pkg_blocked() is called on every verify and every
   launch, and each KVDB open is a flash transaction. Refreshed only when a new
   list is applied — nothing else can change it. */
static uint8_t s_bl[BL_MAX * 8];
static int     s_bl_len = -1;   /* -1 = not read yet */

static void bl_load(void)
{
    share_prefs_t *p;
    int n;

    if (s_bl_len >= 0)
        return;
    s_bl_len = 0;
    p = share_prefs_open(BL_PREFS, SHAREPREFS_MODE_PRIVATE);
    if (!p)
        return;
    n = share_prefs_get_block(p, BL_KEY, s_bl, (int32_t)sizeof(s_bl));
    share_prefs_close(p);
    if (n > 0)
        s_bl_len = n;
}

int32_t skai_pkg_blocklist_seq(void)
{
    share_prefs_t *p = share_prefs_open(BL_PREFS, SHAREPREFS_MODE_PRIVATE);
    int32_t seq;

    if (!p)
        return -1;
    seq = share_prefs_get_int(p, BL_SEQ_KEY, -1);
    share_prefs_close(p);
    return seq;
}

bool skai_pkg_blocked(const char *keyid)
{
    uint8_t want[8];

    if (!keyid || b64u_decode(keyid, (uint32_t)strlen(keyid), want, sizeof(want)) != 8)
        return false;

    bl_load();
    for (int i = 0; i + 8 <= s_bl_len; i += 8)
        if (memcmp(&s_bl[i], want, 8) == 0)
            return true;
    return false;
}

skai_pkg_result_t skai_pkg_blocklist_apply(const char *json, uint32_t len)
{
    cJSON *root, *j;
    uint8_t list[BL_MAX * 8];
    int32_t seq, stored;
    int n = 0;
    share_prefs_t *p;

    if (!json || len == 0)
        return SKAI_PKG_ERR_PARSE;

    root = cJSON_ParseWithLength(json, len);
    if (!root)
        return SKAI_PKG_ERR_PARSE;

    j = cJSON_GetObjectItem(root, "skai_blocklist");
    if (!cJSON_IsNumber(j) || j->valueint != 1)
        goto parse_error;

    j = cJSON_GetObjectItem(root, "seq");
    if (!cJSON_IsNumber(j) || j->valueint < 0)
        goto parse_error;
    seq = (int32_t)j->valueint;

    j = cJSON_GetObjectItem(root, "revoked");
    if (!cJSON_IsArray(j))
        goto parse_error;

    for (cJSON *e = j->child; e && n + 8 <= (int)sizeof(list); e = e->next)
    {
        cJSON *k = cJSON_GetObjectItem(e, "keyid");
        if (!cJSON_IsString(k) ||
                b64u_decode(k->valuestring, (uint32_t)strlen(k->valuestring),
                            &list[n], 8) != 8)
            goto parse_error;
        n += 8;
    }
    cJSON_Delete(root);

    p = share_prefs_open(BL_PREFS, SHAREPREFS_MODE_PRIVATE);
    if (!p)
        return SKAI_PKG_ERR_STORAGE;

    /* Monotonic: a replayed older list must not resurrect a revoked key. */
    stored = share_prefs_get_int(p, BL_SEQ_KEY, -1);
    if (seq <= stored)
    {
        share_prefs_close(p);
        LOG_I("blocklist seq %d <= stored %d, ignored", (int)seq, (int)stored);
        return SKAI_PKG_OK;
    }
    /* An empty list must REMOVE the record: writing a zero-length block leaves
       the previous one in place, which would mean a revocation could never be
       lifted. */
    if (n > 0)
        share_prefs_set_block(p, BL_KEY, list, n);
    else
        share_prefs_remove(p, BL_KEY);
    share_prefs_set_int(p, BL_SEQ_KEY, seq);
    share_prefs_close(p);

    memcpy(s_bl, list, (size_t)n);
    s_bl_len = n;
    LOG_I("blocklist seq %d: %d revoked keyid(s)", (int)seq, n / 8);
    return SKAI_PKG_OK;

parse_error:
    cJSON_Delete(root);
    return SKAI_PKG_ERR_PARSE;
}

/* ──────────────────────────────── verify ───────────────────────────────── */

const char *skai_pkg_result_name(skai_pkg_result_t r)
{
    switch (r)
    {
    case SKAI_PKG_OK:            return "ok";
    case SKAI_PKG_ERR_PARSE:     return "manifest-invalid";
    case SKAI_PKG_ERR_VERSION:   return "firmware-too-old";
    case SKAI_PKG_ERR_KEYID:     return "keyid-mismatch";
    case SKAI_PKG_ERR_DIGEST:    return "payload-digest";
    case SKAI_PKG_ERR_SIGNATURE: return "signature";
    case SKAI_PKG_ERR_BLOCKED:   return "publisher-revoked";
    case SKAI_PKG_ERR_STORAGE:   return "storage";
    case SKAI_PKG_ERR_LIMIT:     return "too-many-or-too-big";
    case SKAI_PKG_ERR_NOTFOUND:  return "not-installed";
    default:                     return "internal";
    }
}

/* ">=1.2" against SKAI_API_MAJOR/MINOR. Hand-parsed rather than sscanf'd so a
   trailing "1.2.3" or "1.2x" is rejected instead of silently truncated. */
static bool version_ok(const char *s)
{
    unsigned maj = 0, min = 0;
    const char *p = s;

    if (!s || s[0] != '>' || s[1] != '=')
        return false;
    p += 2;
    if (*p < '0' || *p > '9')
        return false;
    while (*p >= '0' && *p <= '9')
        maj = maj * 10 + (unsigned)(*p++ - '0');
    if (*p++ != '.')
        return false;
    if (*p < '0' || *p > '9')
        return false;
    while (*p >= '0' && *p <= '9')
        min = min * 10 + (unsigned)(*p++ - '0');
    if (*p != '\0')
        return false;

    return SKAI_API_SATISFIES(maj, min);
}

static bool copy_field(char *dst, uint32_t cap, const cJSON *o, const char *key)
{
    const cJSON *j = cJSON_GetObjectItem(o, key);

    if (!cJSON_IsString(j) || !j->valuestring)
        return false;
    if (strlen(j->valuestring) >= cap)
        return false;   /* longer than the schema allows -> not a v1 manifest */
    strcpy(dst, j->valuestring);
    return true;
}

/* The digest the signature covers. Fed to SHA-256 field by field rather than
 * built in a buffer: it is the same bytes, and there is no 1.5 KB temporary to
 * size wrongly. Capabilities are sorted bytewise here — the manifest may list
 * them in any order, but only one order is signed. */
static void hash_signed_input(mbedtls_sha256_context *c,
                              const skai_pkg_info_t *info,
                              const char *payload_type,
                              const char *payload_sha_b64)
{
    const char *sorted[SKAI_PKG_CAPS_MAX];
    char sizebuf[12];
    const char lf = '\n';

    for (uint16_t i = 0; i < info->n_caps; i++)
        sorted[i] = info->caps[i];
    for (uint16_t i = 1; i < info->n_caps; i++)   /* insertion sort, n <= 32 */
    {
        const char *v = sorted[i];
        int j = (int)i - 1;
        while (j >= 0 && strcmp(sorted[j], v) > 0)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = v;
    }

    rt_snprintf(sizebuf, sizeof(sizebuf), "%u", (unsigned)info->payload_size);

#define FIELD(s) do { \
        mbedtls_sha256_update(c, (const uint8_t *)(s), (uint32_t)strlen(s)); \
        mbedtls_sha256_update(c, (const uint8_t *)&lf, 1); \
    } while (0)

    FIELD("skaiapp-v1");
    FIELD(info->keyid);
    FIELD(info->app_id);
    FIELD(info->version);
    FIELD(payload_type);
    FIELD(sizebuf);
    FIELD(payload_sha_b64);

    for (uint16_t i = 0; i < info->n_caps; i++)
    {
        if (i)
            mbedtls_sha256_update(c, (const uint8_t *)",", 1);
        mbedtls_sha256_update(c, (const uint8_t *)sorted[i],
                                  (uint32_t)strlen(sorted[i]));
    }
    mbedtls_sha256_update(c, (const uint8_t *)&lf, 1);
#undef FIELD
}

static skai_pkg_result_t ecdsa_ok(const uint8_t pub[64], const uint8_t sig[64],
                                  const uint8_t digest[32])
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi r, s;
    uint8_t point[65];
    int ret = -1;

    mbedtls_ecp_group_init(&grp);
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    point[0] = 0x04;                  /* uncompressed; the schema carries X||Y */
    memcpy(&point[1], pub, 64);

    if (mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_SECP256R1) != 0)
        goto out;
    if (mbedtls_ecp_point_read_binary(&grp, &Q, point, sizeof(point)) != 0)
        goto out;
    /* read_binary does not check the point is on the curve; an off-curve key is
       an attack surface, not a typo. */
    if (mbedtls_ecp_check_pubkey(&grp, &Q) != 0)
        goto out;
    if (mbedtls_mpi_read_binary(&r, &sig[0], 32) != 0 ||
            mbedtls_mpi_read_binary(&s, &sig[32], 32) != 0)
        goto out;

    ret = mbedtls_ecdsa_verify(&grp, digest, 32, &Q, &r, &s);

out:
    mbedtls_mpi_free(&s);
    mbedtls_mpi_free(&r);
    mbedtls_ecp_point_free(&Q);
    mbedtls_ecp_group_free(&grp);
    return (ret == 0) ? SKAI_PKG_OK : SKAI_PKG_ERR_SIGNATURE;
}

skai_pkg_result_t skai_pkg_verify(const char *manifest, uint32_t mlen,
                                  const uint8_t *payload, uint32_t plen,
                                  skai_pkg_info_t *out)
{
    cJSON *root = NULL, *app, *pub, *caps, *pay, *j;
    uint8_t pubkey[64], sig[64], keyid[8], want_sha[32], got[32];
    char pay_sha_b64[44], pay_type[16];
    mbedtls_sha256_context c;
    skai_pkg_result_t res = SKAI_PKG_ERR_PARSE;

    if (!manifest || !out || mlen == 0 || mlen > SKAI_PKG_MANIFEST_MAX)
        return SKAI_PKG_ERR_PARSE;
    if (!payload && plen)
        return SKAI_PKG_ERR_PARSE;

    memset(out, 0, sizeof(*out));

    root = cJSON_ParseWithLength(manifest, mlen);
    if (!root)
        return SKAI_PKG_ERR_PARSE;

    /* Top-level key sniffing, per the schema: "skai" is v1. A legacy bare
       package has "skaiapp" instead and belongs to skaiapp_store, not here. */
    j = cJSON_GetObjectItem(root, "skai");
    if (!cJSON_IsString(j))
        goto done;

    app = cJSON_GetObjectItem(root, "app");
    pub = cJSON_GetObjectItem(root, "publisher");
    caps = cJSON_GetObjectItem(root, "capabilities");
    pay = cJSON_GetObjectItem(root, "payload");
    if (!cJSON_IsObject(app) || !cJSON_IsObject(pub) ||
            !cJSON_IsArray(caps) || !cJSON_IsObject(pay))
        goto done;

    if (!copy_field(out->app_id, sizeof(out->app_id), app, "id") ||
            !copy_field(out->name, sizeof(out->name), app, "name") ||
            !copy_field(out->version, sizeof(out->version), app, "version") ||
            !copy_field(out->keyid, sizeof(out->keyid), pub, "keyid") ||
            !copy_field(pay_type, sizeof(pay_type), pay, "type") ||
            !copy_field(pay_sha_b64, sizeof(pay_sha_b64), pay, "sha256"))
        goto done;

    if (out->app_id[0] == '\0')
        goto done;

    {
        const cJSON *alg = cJSON_GetObjectItem(pub, "alg");
        const cJSON *pk  = cJSON_GetObjectItem(pub, "pubkey");
        const cJSON *sg  = cJSON_GetObjectItem(root, "sig");
        const cJSON *sz  = cJSON_GetObjectItem(pay, "size");

        if (!cJSON_IsString(alg) || strcmp(alg->valuestring, "ES256") != 0)
            goto done;
        if (!cJSON_IsString(pk) ||
                b64u_decode(pk->valuestring, (uint32_t)strlen(pk->valuestring),
                            pubkey, sizeof(pubkey)) != 64)
            goto done;
        if (!cJSON_IsString(sg) ||
                b64u_decode(sg->valuestring, (uint32_t)strlen(sg->valuestring),
                            sig, sizeof(sig)) != 64)
            goto done;
        if (b64u_decode(out->keyid, (uint32_t)strlen(out->keyid),
                        keyid, sizeof(keyid)) != 8)
            goto done;
        if (b64u_decode(pay_sha_b64, (uint32_t)strlen(pay_sha_b64),
                        want_sha, sizeof(want_sha)) != 32)
            goto done;
        if (!cJSON_IsNumber(sz) || sz->valuedouble < 1 ||
                sz->valuedouble > SKAI_PKG_PAYLOAD_MAX)
            goto done;
        out->payload_size = (uint32_t)sz->valuedouble;
    }

    if (strcmp(pay_type, "js") == 0)
        out->is_js = true;
    else if (strcmp(pay_type, "declarative") != 0)
        goto done;

    for (cJSON *e = caps->child; e; e = e->next)
    {
        if (!cJSON_IsString(e) || !e->valuestring)
            goto done;
        if (out->n_caps >= SKAI_PKG_CAPS_MAX ||
                strlen(e->valuestring) >= SKAI_PKG_CAP_MAX)
            goto done;
        strcpy(out->caps[out->n_caps++], e->valuestring);
    }

    /* Refused at INSTALL with a reason, never mid-run: an app that needs a
       newer firmware must not get halfway into a screen before finding out. */
    if (!version_ok(j->valuestring))
    {
        res = SKAI_PKG_ERR_VERSION;
        goto done;
    }

    /* keyid must be derived from the key, or it is just a label an attacker
       chooses — and TOFU, the blocklist and the log tag all key off it. */
    sha256(pubkey, sizeof(pubkey), got);
    if (memcmp(got, keyid, 8) != 0)
    {
        res = SKAI_PKG_ERR_KEYID;
        goto done;
    }

    if (skai_pkg_blocked(out->keyid))
    {
        res = SKAI_PKG_ERR_BLOCKED;
        goto done;
    }

    if (plen != out->payload_size)
    {
        res = SKAI_PKG_ERR_DIGEST;
        goto done;
    }
    sha256(payload, plen, got);
    if (memcmp(got, want_sha, 32) != 0)
    {
        res = SKAI_PKG_ERR_DIGEST;
        goto done;
    }

    mbedtls_sha256_init(&c);
    mbedtls_sha256_starts(&c, 0);
    hash_signed_input(&c, out, pay_type, pay_sha_b64);
    mbedtls_sha256_finish(&c, got);
    mbedtls_sha256_free(&c);

    res = ecdsa_ok(pubkey, sig, got);

done:
    cJSON_Delete(root);
    if (res != SKAI_PKG_OK)
        memset(out, 0, sizeof(*out));
    return res;
}

/* ─────────────────────────────── storage ───────────────────────────────── */

/* /skaiapp/<keyid>/<app_id>/{manifest.json,payload}
 *
 * keyid is part of the PATH, which is what makes (keyid, app_id) the primary
 * key the ADR asks for: a second publisher claiming the same app id lands in a
 * different directory and cannot overwrite the first. That also gives TOFU for
 * free and without a second source of truth — there is no keyid record to
 * disagree with the filesystem. Both components are constrained by the schema
 * to [A-Za-z0-9_-], so neither can walk out of PKG_DIR. */
static bool pkg_dir(char *buf, uint32_t cap, const char *keyid, const char *app_id)
{
    /* Defence in depth: the schema patterns already exclude these, but this is
       the function that turns untrusted strings into a path. */
    for (const char *p = keyid; *p; p++)
        if (*p == '/' || *p == '\\' || *p == '.')
            return false;
    for (const char *p = app_id; *p; p++)
        if (*p == '/' || *p == '\\' || *p == '.')
            return false;
    if (!keyid[0] || !app_id[0])
        return false;
    return rt_snprintf(buf, cap, PKG_DIR "/%s/%s", keyid, app_id) < (int)cap;
}

static int write_file(const char *path, const uint8_t *data, uint32_t len)
{
    int fd = skai_fopen(path, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0666);
    int n;

    if (fd < 0)
        return -1;
    n = skai_fwrite(fd, data, (int)len);
    skai_fclose(fd);
    return (n == (int)len) ? 0 : -1;
}

static int read_file(const char *path, uint8_t *buf, uint32_t cap, uint32_t *out_len)
{
    int fd = skai_fopen(path, O_RDONLY, 0);
    int n;

    if (fd < 0)
        return -1;
    n = skai_fread(fd, buf, (int)cap);
    skai_fclose(fd);
    if (n < 0)
        return -1;
    *out_len = (uint32_t)n;
    return 0;
}

skai_pkg_result_t skai_pkg_install(const char *manifest, uint32_t mlen,
                                   const uint8_t *payload, uint32_t plen,
                                   skai_pkg_info_t *out)
{
    char dir[96], path[128];
    skai_pkg_result_t res = skai_pkg_verify(manifest, mlen, payload, plen, out);

    if (res != SKAI_PKG_OK)
        return res;

    /* The JS host holds the source in a fixed buffer, so a payload it could not
       run is refused HERE rather than truncated at launch. */
    if (out->is_js && plen >= SKAI_PKG_JS_SRC_MAX)
    {
        LOG_W("%s: js payload %u B exceeds %u", out->app_id,
              (unsigned)plen, (unsigned)SKAI_PKG_JS_SRC_MAX);
        return SKAI_PKG_ERR_LIMIT;
    }

    if (!pkg_dir(dir, sizeof(dir), out->keyid, out->app_id))
        return SKAI_PKG_ERR_STORAGE;

    mkdir(PKG_DIR, 0x777);
    rt_snprintf(path, sizeof(path), PKG_DIR "/%s", out->keyid);
    mkdir(path, 0x777);
    mkdir(dir, 0x777);          /* all three may already exist */

    /* Payload first, manifest last: the manifest is what the scan looks for, so
       a power loss mid-install leaves a directory the launcher ignores rather
       than an app with no code. */
    rt_snprintf(path, sizeof(path), "%s/payload", dir);
    if (write_file(path, payload, plen) != 0)
        return SKAI_PKG_ERR_STORAGE;

    rt_snprintf(path, sizeof(path), "%s/manifest.json", dir);
    if (write_file(path, (const uint8_t *)manifest, mlen) != 0)
        return SKAI_PKG_ERR_STORAGE;

    LOG_I("installed %s/%s v%s (%u caps, %u B)", out->keyid, out->app_id,
          out->version, (unsigned)out->n_caps, (unsigned)plen);
    return SKAI_PKG_OK;
}

/* Walk /skaiapp/<keyid>/<app_id>/manifest.json. Directory entries are probed by
 * opening rather than by d_type, which is not uniform across the FS ports here.
 *
 * ponytail: rescans on every call instead of keeping a resident table. There
 * are at most SKAI_PKG_SLOTS packages and this runs when a launcher opens, not
 * per frame. Add the table when the list is on a scrolling screen. */
static int pkg_walk(int want_idx, skai_pkg_info_t *out,
                    const char *want_keyid, const char *want_app)
{
    DIR *d1, *d2;
    struct dirent *e1, *e2;
    char path[128];
    static uint8_t mbuf[SKAI_PKG_MANIFEST_MAX];
    int found = 0;

    d1 = opendir(PKG_DIR);
    if (!d1)
        return 0;

    while ((e1 = readdir(d1)) != NULL && found < SKAI_PKG_SLOTS)
    {
        if (e1->d_name[0] == '.')
            continue;
        rt_snprintf(path, sizeof(path), PKG_DIR "/%s", e1->d_name);
        d2 = opendir(path);
        if (!d2)
            continue;   /* a legacy /skaiapp/<id>.json file, not a keyid dir */

        while ((e2 = readdir(d2)) != NULL && found < SKAI_PKG_SLOTS)
        {
            uint32_t mlen = 0;
            skai_pkg_info_t info;

            if (e2->d_name[0] == '.')
                continue;
            rt_snprintf(path, sizeof(path), PKG_DIR "/%s/%s/manifest.json",
                        e1->d_name, e2->d_name);
            if (read_file(path, mbuf, sizeof(mbuf), &mlen) != 0 || mlen == 0)
                continue;

            /* Parsed, not re-verified: the signature was checked at install and
               the schema pins it there. Enough is read to name the app. */
            memset(&info, 0, sizeof(info));
            {
                cJSON *root = cJSON_ParseWithLength((const char *)mbuf, mlen);
                cJSON *app = root ? cJSON_GetObjectItem(root, "app") : NULL;
                cJSON *pub = root ? cJSON_GetObjectItem(root, "publisher") : NULL;
                cJSON *pay = root ? cJSON_GetObjectItem(root, "payload") : NULL;
                bool ok = app && pub && pay
                          && copy_field(info.app_id, sizeof(info.app_id), app, "id")
                          && copy_field(info.name, sizeof(info.name), app, "name")
                          && copy_field(info.version, sizeof(info.version), app, "version")
                          && copy_field(info.keyid, sizeof(info.keyid), pub, "keyid");
                if (ok)
                {
                    cJSON *t = cJSON_GetObjectItem(pay, "type");
                    info.is_js = cJSON_IsString(t) && strcmp(t->valuestring, "js") == 0;
                }
                cJSON_Delete(root);
                if (!ok)
                    continue;
            }
            /* The directory names are authoritative; a manifest that disagrees
               with where it sits is not something to guess about. */
            if (strcmp(info.keyid, e1->d_name) != 0 ||
                    strcmp(info.app_id, e2->d_name) != 0)
                continue;

            if (want_keyid)
            {
                if (strcmp(info.keyid, want_keyid) == 0 &&
                        strcmp(info.app_id, want_app) == 0)
                {
                    if (out) *out = info;
                    closedir(d2);
                    closedir(d1);
                    return 1;
                }
            }
            else if (want_idx >= 0 && found == want_idx)
            {
                if (out) *out = info;
                closedir(d2);
                closedir(d1);
                return 1;
            }
            found++;
        }
        closedir(d2);
    }
    closedir(d1);
    return (want_idx < 0 && !want_keyid) ? found : 0;
}

int skai_pkg_count(void)
{
    return pkg_walk(-1, NULL, NULL, NULL);
}

bool skai_pkg_at(int idx, skai_pkg_info_t *out)
{
    if (idx < 0 || !out)
        return false;
    return pkg_walk(idx, out, NULL, NULL) == 1;
}

skai_pkg_result_t skai_pkg_remove(const char *keyid, const char *app_id)
{
    char dir[96], path[128];

    if (!keyid || !app_id || !pkg_dir(dir, sizeof(dir), keyid, app_id))
        return SKAI_PKG_ERR_NOTFOUND;

    rt_snprintf(path, sizeof(path), "%s/manifest.json", dir);
    if (unlink(path) != 0)
        return SKAI_PKG_ERR_NOTFOUND;
    rt_snprintf(path, sizeof(path), "%s/payload", dir);
    unlink(path);
    rmdir(dir);
    /* The keyid directory is left: another app by the same publisher may live
       in it, and an empty one costs a directory entry. */
    LOG_I("removed %s/%s", keyid, app_id);
    return SKAI_PKG_OK;
}

/* ──────────────────────────────── launch ───────────────────────────────── */

#include "gui_app_fwk.h"
#include "ui_handler.h"

extern void skaijs_set_source(const char *src, const skai_js_policy_t *policy);

/* The policy's capability array is borrowed by the runtime for the life of the
   app, so both it and the strings it points at have to outlive this call. One
   app is on screen at a time, which is what makes a single static correct. */
static skai_pkg_info_t s_live;
static const char     *s_live_caps[SKAI_PKG_CAPS_MAX];
static char            s_live_src[SKAI_PKG_JS_SRC_MAX];

skai_pkg_result_t skai_pkg_launch(const char *keyid, const char *app_id)
{
    char dir[96], path[128];
    uint32_t plen = 0;
    uint8_t digest[32];
    skai_js_policy_t pol;

    if (!keyid || !app_id)
        return SKAI_PKG_ERR_NOTFOUND;
    if (pkg_walk(-1, &s_live, keyid, app_id) != 1)
        return SKAI_PKG_ERR_NOTFOUND;
    if (!s_live.is_js)
        return SKAI_PKG_ERR_PARSE;      /* declarative packages are not ours */

    /* Revoked publishers are stopped at launch too, not only at install — the
       blocklist arrives long after the app did. */
    if (skai_pkg_blocked(keyid))
        return SKAI_PKG_ERR_BLOCKED;

    if (!pkg_dir(dir, sizeof(dir), keyid, app_id))
        return SKAI_PKG_ERR_NOTFOUND;
    rt_snprintf(path, sizeof(path), "%s/payload", dir);
    if (read_file(path, (uint8_t *)s_live_src, sizeof(s_live_src) - 1, &plen) != 0)
        return SKAI_PKG_ERR_STORAGE;
    s_live_src[plen] = '\0';

    /* Re-hash the payload but not the signature: the signature binds the
       PUBLISHER and the schema verifies it once, at install; the digest binds
       the BYTES and catches a swapped or truncated file for the price of a
       hash. Re-running ECDSA on every launch would buy little more. */
    {
        uint32_t mlen = 0;
        static uint8_t mbuf[SKAI_PKG_MANIFEST_MAX];
        cJSON *root, *pay, *j;
        uint8_t want[32];
        bool ok;

        rt_snprintf(path, sizeof(path), "%s/manifest.json", dir);
        if (read_file(path, mbuf, sizeof(mbuf), &mlen) != 0)
            return SKAI_PKG_ERR_STORAGE;
        root = cJSON_ParseWithLength((const char *)mbuf, mlen);
        pay = root ? cJSON_GetObjectItem(root, "payload") : NULL;
        j = pay ? cJSON_GetObjectItem(pay, "sha256") : NULL;
        ok = cJSON_IsString(j) &&
             b64u_decode(j->valuestring, (uint32_t)strlen(j->valuestring),
                         want, sizeof(want)) == 32;
        if (ok)
        {
            /* Capabilities come from the manifest, never from the payload. */
            cJSON *caps = cJSON_GetObjectItem(root, "capabilities");
            s_live.n_caps = 0;
            for (cJSON *e = caps ? caps->child : NULL; e; e = e->next)
            {
                if (!cJSON_IsString(e) || s_live.n_caps >= SKAI_PKG_CAPS_MAX ||
                        strlen(e->valuestring) >= SKAI_PKG_CAP_MAX)
                {
                    ok = false;
                    break;
                }
                strcpy(s_live.caps[s_live.n_caps++], e->valuestring);
            }
        }
        cJSON_Delete(root);
        if (!ok)
            return SKAI_PKG_ERR_PARSE;

        sha256((const uint8_t *)s_live_src, plen, digest);
        if (memcmp(digest, want, 32) != 0)
        {
            LOG_W("%s/%s: payload no longer matches its manifest", keyid, app_id);
            return SKAI_PKG_ERR_DIGEST;
        }
    }

    skai_js_policy_init(&pol, s_live.app_id, s_live.keyid);
    for (uint16_t i = 0; i < s_live.n_caps; i++)
        s_live_caps[i] = s_live.caps[i];
    pol.caps = s_live_caps;
    pol.n_caps = s_live.n_caps;

    skaijs_set_source(s_live_src, &pol);
    LOG_I("launching %s/%s v%s", keyid, app_id, s_live.version);
    gui_app_run(APP_ID_SKAIJS);
    return SKAI_PKG_OK;
}
