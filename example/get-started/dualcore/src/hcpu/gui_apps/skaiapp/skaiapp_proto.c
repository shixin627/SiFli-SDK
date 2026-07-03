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
 */
#include <string.h>
#include <stdlib.h>
#include <rtthread.h>
#include "cJSON.h"
#include "skaiapp_pkg.h"
#include "skaiapp_store.h"
#include "communicate_task.h"

#define DBG_TAG "skaiapp.proto"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define CHUNK_RAW_MAX 3072
#define REASM_CAP     (SKAIAPP_PKG_MAX_BYTES + 64)

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
    if (s_re.buf != NULL)
    {
        rt_free(s_re.buf);
    }
    memset(&s_re, 0, sizeof(s_re));
}

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

    if (!wire_id_ok(id) || seq < 0 || n < 1 || n > 8 || chunk == NULL)
    {
        ack = SKAIAPP_ACK_PARSE;
        session_reset();
        goto out;
    }

    if (seq == 0)
    {
        session_reset();
        const cJSON *jcrc = cJSON_GetObjectItem(root, "crc");
        if (jcrc == NULL || !cJSON_IsNumber(jcrc))
        {
            ack = SKAIAPP_ACK_PARSE;
            goto out;
        }
        s_re.buf = rt_malloc(REASM_CAP);
        if (s_re.buf == NULL)
        {
            ack = SKAIAPP_ACK_STORAGE;
            goto out;
        }
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
    const cJSON *jid = cJSON_GetObjectItem(root, "id");
    if (jid != NULL && cJSON_IsString(jid) && jid->valuestring != NULL)
    {
        strncpy(id, jid->valuestring, sizeof(id) - 1);
    }
    cJSON_Delete(root);

    if (!wire_id_ok(id))
    {
        return;
    }
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
#endif /* FINSH_USING_MSH */
