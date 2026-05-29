#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
oss_upload.py — upload release artifacts to Aliyun OSS (pure stdlib, no SDK).

Signing mirrors SkaiLink EXACTLY (lib/shared/cloud/storage/aliyun_oss_service.dart,
_SignatureGenerator.generateSignature), so the phone app and this uploader agree:

    StringToSign = "{METHOD}\n{Content-MD5}\n{Content-Type}\n{Date}\n/{bucket}/{key}"
    Signature    = base64( HMAC-SHA1( AccessKeySecret, StringToSign ) )
    Authorization: "OSS {AccessKeyId}:{Signature}"

NOTE: SkaiLink's scheme omits the CanonicalizedOSSHeaders block entirely (no
x-oss-* lines in the StringToSign). We do the same on purpose — matching the
existing, working phone implementation beats following the generic OSS doc.

Object key layout for the watch release (per project convention):
    info.json     -> skaiwatch/<board>/info.json          e.g. skaiwatch/29/info.json
    watchOS.zip   -> skaiwatch/<board>/<version>/watchOS.zip
                                                           e.g. skaiwatch/29/1.1.60/watchOS.zip

Credentials are read from (file first, then env overrides), NEVER hard-coded.
Key names match SkaiLink's .env.json so you can point straight at it:
    OSS_ENDPOINT  OSS_BUCKET  ALIYUN_ACCESS_KEY_ID  ALIYUN_ACCESS_KEY_SECRET
    OSS_SECURITY_TOKEN (optional, STS)
Default file: oss_credentials.json next to this script, or env OSS_CREDENTIALS_FILE.

Usage (CLI):
    python oss_upload.py --selftest          # verify signing vs SkaiLink's scheme
    python oss_upload.py --check             # show resolved endpoint/bucket (no secrets)
    python oss_upload.py put <local> <key>   # upload one file to <key>
"""

import os
import io
import sys
import json
import hmac
import base64
import hashlib
import mimetypes
from email.utils import formatdate

try:
    import urllib.request as _urlreq
    import urllib.error as _urlerr
except Exception:  # pragma: no cover
    _urlreq = None
    _urlerr = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CRED_FILE = os.path.join(SCRIPT_DIR, "oss_credentials.json")

# Maps our internal field -> the credential key name (matches SkaiLink .env.json).
CRED_KEYS = {
    "endpoint": "OSS_ENDPOINT",
    "bucket": "OSS_BUCKET",
    "access_key_id": "ALIYUN_ACCESS_KEY_ID",
    "access_key_secret": "ALIYUN_ACCESS_KEY_SECRET",
    "security_token": "OSS_SECURITY_TOKEN",
}


class OssError(Exception):
    pass


# --- credential loading --------------------------------------------------

def load_credentials():
    """Return dict endpoint/bucket/access_key_id/access_key_secret/security_token.
    File first (default oss_credentials.json or OSS_CREDENTIALS_FILE), then env
    vars override. Raises OssError if the required four are missing."""
    creds = {}
    cred_file = os.environ.get("OSS_CREDENTIALS_FILE", DEFAULT_CRED_FILE)
    if os.path.exists(cred_file):
        with io.open(cred_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        for field, name in CRED_KEYS.items():
            if name in data and str(data[name]).strip():
                creds[field] = str(data[name]).strip()
    for field, name in CRED_KEYS.items():
        v = os.environ.get(name, "").strip()
        if v:
            creds[field] = v

    # endpoint may carry an https:// prefix in .env.json — strip it.
    if creds.get("endpoint"):
        creds["endpoint"] = creds["endpoint"].replace("https://", "").replace("http://", "")

    missing = [CRED_KEYS[f] for f in ("endpoint", "bucket", "access_key_id",
                                      "access_key_secret") if not creds.get(f)]
    if missing:
        raise OssError(
            "缺少 OSS 認證: %s。請設定環境變數,或建立 %s(可直接複製 SkaiLink 的 .env.json)。"
            % (", ".join(missing), DEFAULT_CRED_FILE))
    creds.setdefault("security_token", "")
    return creds


# --- signing (mirrors SkaiLink _SignatureGenerator.generateSignature) ----

def build_string_to_sign(method, content_md5, content_type, date, bucket, key):
    """SkaiLink's exact format: no canonicalized-headers block."""
    return "%s\n%s\n%s\n%s\n/%s/%s" % (method, content_md5, content_type,
                                       date, bucket, key)


def sign(access_key_secret, string_to_sign):
    digest = hmac.new(access_key_secret.encode("utf-8"),
                      string_to_sign.encode("utf-8"), hashlib.sha1).digest()
    return base64.b64encode(digest).decode("ascii")


def _content_type_for(name):
    if name.lower().endswith(".json"):
        return "application/json"
    if name.lower().endswith(".zip"):
        return "application/octet-stream"  # SkaiLink default for unknown ext
    return mimetypes.guess_type(name)[0] or "application/octet-stream"


# --- upload --------------------------------------------------------------

def put_object(creds, key, local_path, date=None, content_type=None,
               dry_run=False):
    """PUT a local file to bucket/key. Returns the object URL on success.
    dry_run=True returns (url, headers, string_to_sign) without sending."""
    if not os.path.exists(local_path):
        raise OssError("找不到檔案: %s" % local_path)
    key = key.lstrip("/")

    with open(local_path, "rb") as f:
        body = f.read()
    content_md5 = base64.b64encode(hashlib.md5(body).digest()).decode("ascii")
    content_type = content_type or _content_type_for(local_path)
    date = date or formatdate(usegmt=True)

    sts = build_string_to_sign("PUT", content_md5, content_type, date,
                               creds["bucket"], key)
    signature = sign(creds["access_key_secret"], sts)

    headers = {
        "Authorization": "OSS %s:%s" % (creds["access_key_id"], signature),
        "Content-MD5": content_md5,
        "Content-Type": content_type,
        "Date": date,
    }
    if creds.get("security_token"):
        headers["x-oss-security-token"] = creds["security_token"]

    url = "https://%s.%s/%s" % (creds["bucket"], creds["endpoint"], key)
    if dry_run:
        return url, headers, sts

    if _urlreq is None:
        raise OssError("此 Python 缺少 urllib,無法上傳。")
    req = _urlreq.Request(url, data=body, method="PUT")
    for k, v in headers.items():
        req.add_header(k, v)
    try:
        with _urlreq.urlopen(req, timeout=120) as resp:
            if 200 <= resp.status < 300:
                return url
            raise OssError("OSS 回應 %s" % resp.status)
    except _urlerr.HTTPError as e:
        detail = ""
        try:
            detail = e.read().decode("utf-8", "replace")[:500]
        except Exception:
            pass
        raise OssError("上傳失敗 HTTP %s: %s\n%s" % (e.code, e.reason, detail))
    except _urlerr.URLError as e:
        raise OssError("連線失敗: %s" % e.reason)


# --- release-specific key helpers ---------------------------------------

def info_json_key(board):
    return "skaiwatch/%s/info.json" % board


def watchos_zip_key(board, version):
    return "skaiwatch/%s/%s/watchOS.zip" % (board, version)


# --- self test (offline) -------------------------------------------------

def selftest():
    """Verify signing reproduces SkaiLink's scheme on a fixed input, and that
    the round-trip of build_string_to_sign matches the Dart string template."""
    out = []

    def add(s):
        out.append(str(s))

    # Fixed inputs -> deterministic signature. The "expected" value below is
    # computed by the same algorithm SkaiLink uses (HMAC-SHA1 over the no-header
    # StringToSign); we assert our two code paths agree and the template is exact.
    secret = "testsecret"
    method, md5v, ctype = "PUT", "abc123==", "application/json"
    date = "Thu, 17 Nov 2005 18:49:58 GMT"
    bucket, key = "mybucket", "skaiwatch/29/info.json"

    sts = build_string_to_sign(method, md5v, ctype, date, bucket, key)
    expected_template = ("PUT\nabc123==\napplication/json\n"
                         "Thu, 17 Nov 2005 18:49:58 GMT\n/mybucket/skaiwatch/29/info.json")
    add("string_to_sign_matches_dart_template: %s" % (sts == expected_template))

    # Independent re-implementation of the HMAC to cross-check sign().
    ref = base64.b64encode(
        hmac.new(secret.encode(), sts.encode(), hashlib.sha1).digest()
    ).decode()
    add("sign_matches_reference_hmac: %s" % (sign(secret, sts) == ref))
    add("  signature: %s" % sign(secret, sts))

    add("info_json_key(29): %s" % info_json_key(29))
    add("watchos_zip_key(29, '1.1.60'): %s" % watchos_zip_key(29, "1.1.60"))

    try:
        c = load_credentials()
        add("credentials_loaded: True (endpoint=%s bucket=%s)"
            % (c["endpoint"], c["bucket"]))
    except OssError as e:
        add("credentials_loaded: False (%s)" % e)

    ok = (sts == expected_template) and (sign(secret, sts) == ref)
    with io.open(os.path.join(SCRIPT_DIR, "_oss_selftest.txt"), "w",
                 encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")
    print("\n".join(out))
    return 0 if ok else 1


def _check():
    try:
        c = load_credentials()
        print("endpoint = %s" % c["endpoint"])
        print("bucket   = %s" % c["bucket"])
        print("key_id   = %s***" % c["access_key_id"][:4])
        print("token    = %s" % ("<set>" if c.get("security_token") else "<none>"))
        return 0
    except OssError as e:
        print(str(e))
        return 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    if "--check" in sys.argv:
        return _check()
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if len(args) == 3 and args[0] == "put":
        try:
            creds = load_credentials()
            print("已上傳: %s" % put_object(creds, args[2], args[1]))
            return 0
        except OssError as e:
            print("錯誤: %s" % e)
            return 1
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
