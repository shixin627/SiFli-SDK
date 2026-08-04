/*
 * Skai SDK — signed app packages: verify, install, launch (ADR-0019 §2.5).
 *
 * The contract is SkaiLink/bridge/skaiapp-manifest.schema.json v1 and nothing
 * here may drift from it: the phone signs, the watch verifies, and a published
 * format is carried forever because external developers cannot reflash.
 *
 * A package is TWO blobs, never one:
 *
 *   manifest  the signing envelope (<= 2048 B of JSON)
 *   payload   the app itself, bound to the manifest only by its SHA-256
 *
 * They are separate so the signature covers the exact transmitted bytes with no
 * re-encoding, and so the firmware needs no JSON canonicalizer — `sig` covers a
 * delimited ASCII byte string (the schema's `signedInput`), not the JSON.
 *
 * Self-signing means the signature grants NOTHING. It buys integrity, publisher
 * continuity and attributability; what an app may actually do is decided
 * entirely by the capability list, which is why that list is inside the digest.
 */
#ifndef SKAI_PKG_H
#define SKAI_PKG_H

#include <stdbool.h>
#include <stdint.h>

/* All from the schema's own patterns, +1 for the terminator. */
#define SKAI_PKG_KEYID_MAX   12   /* base64url of 8 bytes  */
#define SKAI_PKG_ID_MAX      25
#define SKAI_PKG_NAME_MAX    17
#define SKAI_PKG_VERSION_MAX 15
#define SKAI_PKG_CAPS_MAX    32
#define SKAI_PKG_CAP_MAX     42   /* "<=15>.<=23>" */
#define SKAI_PKG_MANIFEST_MAX 2048
#define SKAI_PKG_PAYLOAD_MAX  (256 * 1024)

/* What the JS host can actually hold. The schema allows a payload up to
 * SKAI_PKG_PAYLOAD_MAX, but the source sits in a fixed buffer in HCPU RAM, so
 * a bigger one is refused AT INSTALL with a reason rather than truncated on the
 * way to the interpreter.
 *
 * 8192 was an arbitrary first number and the weather reproduction outgrew it at
 * 8.7 KB — commented source, which is what a hand-written app looks like before
 * anyone minifies it. 16 KB against 800 KB of HCPU SRAM is a fair trade for not
 * making "write fewer comments" a packaging constraint. Past this the buffer
 * should move to PSRAM rather than grow again. */
#define SKAI_PKG_JS_SRC_MAX  16384

/* How many installed packages the launcher will enumerate. */
#define SKAI_PKG_SLOTS       16

/* File I/O that works on BOTH targets. dfs_posix.h says it plainly: under WIN32
 * "MSVC could not replace system CRT functions", so plain open()/read()/write()
 * bind to the C runtime and look for the path on the host disk instead of in
 * the simulated filesystem — they fail, silently and always. dfs exports
 * rt_open()/rt_read()/... there instead, while opendir/readdir/mkdir keep their
 * names. Anything that touches files from both the watch and the simulator has
 * to go through these. */
#ifdef WIN32
    #define skai_fopen  rt_open
    #define skai_fread  rt_read
    #define skai_fwrite rt_write
    #define skai_fclose rt_close
#else
    #define skai_fopen  open
    #define skai_fread  read
    #define skai_fwrite write
    #define skai_fclose close
#endif

/* Every failure is separately reportable: "install failed" is not something a
 * developer with no debugger, or a user with no shell, can act on. */
typedef enum
{
    SKAI_PKG_OK = 0,
    SKAI_PKG_ERR_PARSE,      /* not a v1 manifest, or a field breaks the schema */
    SKAI_PKG_ERR_VERSION,    /* "skai" range not satisfied by this firmware    */
    SKAI_PKG_ERR_KEYID,      /* keyid is not SHA-256(pubkey)[0:8]              */
    SKAI_PKG_ERR_DIGEST,     /* payload does not match payload.sha256          */
    SKAI_PKG_ERR_SIGNATURE,  /* ECDSA P-256 verify failed                      */
    SKAI_PKG_ERR_BLOCKED,    /* publisher keyid is revoked                     */
    SKAI_PKG_ERR_STORAGE,    /* filesystem refused                             */
    SKAI_PKG_ERR_LIMIT,      /* no free slot                                   */
    SKAI_PKG_ERR_NOTFOUND,   /* no such (keyid, app_id)                        */
} skai_pkg_result_t;

const char *skai_pkg_result_name(skai_pkg_result_t r);

/* What a verified manifest says. Roughly 1.4 KB — put it in a static or the
   caller's own storage, not on an RT-Thread thread stack. */
typedef struct
{
    char     keyid[SKAI_PKG_KEYID_MAX];
    char     app_id[SKAI_PKG_ID_MAX];
    char     name[SKAI_PKG_NAME_MAX];
    char     version[SKAI_PKG_VERSION_MAX];
    bool     is_js;             /* payload.type: js vs declarative */
    uint32_t payload_size;
    uint16_t n_caps;
    char     caps[SKAI_PKG_CAPS_MAX][SKAI_PKG_CAP_MAX];
} skai_pkg_info_t;

/* Verify only — no filesystem, no state change. Separate from install so the
   hostile cases can be tested without leaving anything behind. */
skai_pkg_result_t skai_pkg_verify(const char *manifest, uint32_t mlen,
                                  const uint8_t *payload, uint32_t plen,
                                  skai_pkg_info_t *out);

/* Verify, then persist under /skaiapp/<keyid>/<app_id>/. Reinstalling the same
   (keyid, app_id) is an update and overwrites; a different keyid can never
   overwrite, because it is part of the path. */
skai_pkg_result_t skai_pkg_install(const char *manifest, uint32_t mlen,
                                   const uint8_t *payload, uint32_t plen,
                                   skai_pkg_info_t *out);

/* Launcher view. Scans the filesystem each call — there are at most
   SKAI_PKG_SLOTS of these and the launcher opens rarely. */
int  skai_pkg_count(void);
bool skai_pkg_at(int idx, skai_pkg_info_t *out);
skai_pkg_result_t skai_pkg_remove(const char *keyid, const char *app_id);

/* Load an installed package and run it. Re-checks the payload digest and the
   blocklist; does NOT re-run the signature, which the schema pins to install
   time. Only `js` packages launch here — declarative ones belong to the
   existing SkaiApp host. */
skai_pkg_result_t skai_pkg_launch(const char *keyid, const char *app_id);

/* Revocation (skaiapp-blocklist.schema.json). A push whose seq is not greater
   than the stored one is ignored, so a replayed list cannot un-revoke a key. */
bool skai_pkg_blocked(const char *keyid);
skai_pkg_result_t skai_pkg_blocklist_apply(const char *json, uint32_t len);
/* What the watch already has, so the phone can skip pushing a list it holds.
   -1 when none has ever been applied. */
int32_t skai_pkg_blocklist_seq(void);

#endif /* SKAI_PKG_H */
