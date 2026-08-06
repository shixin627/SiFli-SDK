#!/usr/bin/env python3
"""gen_dispatch.py -- project SKAI_EXPORT() annotations into every consumer.

One source of truth: the annotated C headers under src/modules/sdk/includes/skai/.
Three projections, all generated, none hand-maintained:

  skai_dispatch_table.inc   firmware dispatch table (X-macro)
  capability-registry.json  phone-side validator: rejects unknown capabilities
                            at build time instead of on the user's wrist
  skai.d.ts                 developer autocomplete / type checking

The point of generating rather than hand-writing is that a hand-written
external API always falls behind the internal C layer. Adding a capability
must stay a one-line change (ADR-0019).

Declarations are restricted to a fixed type vocabulary. Anything outside it
is a hard error: an exported capability the generator cannot project is worse
than one that does not exist.

    python tools/sdk/gen_dispatch.py                 # generate
    python tools/sdk/gen_dispatch.py --selftest      # check the parser
"""
import argparse
import json
import re
import sys
from pathlib import Path

# (dispatch tag, TypeScript type)
RET_TYPES = {
    "void": ("V", "void"),
    "bool": ("B", "boolean"),
    "int32_t": ("I", "number"),
    "uint32_t": ("U", "number"),
}
ARG_TYPES = {
    "bool": ("B", "boolean"),
    "int32_t": ("I", "number"),
    "uint32_t": ("U", "number"),
    "const char *": ("S", "string"),
}
TIERS = {"SKAI_T1", "SKAI_T2", "SKAI_T2_MIC", "SKAI_T3"}
THREADS = {"SKAI_THREAD_ANY", "SKAI_THREAD_LVGL", "SKAI_THREAD_APP"}

# Must stay in sync with skaiapp-manifest.schema.json capabilities[].pattern.
CAP_RE = re.compile(r"^[a-z][a-z0-9]{0,15}\.[a-z][a-z0-9_]{0,23}$")

EXPORT_RE = re.compile(
    r'SKAI_EXPORT\(\s*"(?P<cap>[^"]*)"\s*,\s*(?P<tier>\w+)\s*,\s*(?P<thread>\w+)\s*'
    r'(?:,\s*"(?P<ui>[^"]*)"\s*)?\)'
    r"\s*(?P<decl>[^;{}]+);",
    re.S,
)


class ExportError(Exception):
    pass


def _split_param(param):
    """'const char *fmt' -> ('const char *', 'fmt')"""
    m = re.match(r"^(?P<type>.*?)(?P<name>[A-Za-z_]\w*)$", param.strip())
    if not m:
        raise ExportError("cannot parse parameter %r" % param)
    return re.sub(r"\s+", " ", m.group("type")).strip().replace(" *", " *"), m.group("name")


COMMENT_RE = re.compile(r"/\*(.*?)\*/", re.S)


def doc_before(text, pos):
    """The comment block immediately above `pos`, cleaned up.

    The headers already carry a sentence or two per capability explaining what
    it returns and when it reports nothing. Those comments ARE the reference;
    re-typing them into a doc page would just create a second copy to forget.
    """
    head = text[:pos].rstrip()
    if not head.endswith("*/"):
        return ""
    start = head.rfind("/*")
    if start < 0:
        return ""
    body = head[start + 2:-2]
    lines = []
    for raw in body.splitlines():
        line = raw.strip().lstrip("*").strip()
        lines.append(line)
    # collapse to paragraphs on blank lines
    paras, cur = [], []
    for line in lines:
        if line:
            cur.append(line)
        elif cur:
            paras.append(" ".join(cur)); cur = []
    if cur:
        paras.append(" ".join(cur))
    # "ponytail:" paragraphs record why something was deliberately kept simple
    # and what the upgrade path is. That is maintainer context; an app developer
    # reading the API reference has no use for it and should not be shown the
    # internals it names.
    paras = [p for p in paras if not p.lower().startswith("ponytail:")]
    return "\n\n".join(paras)


def parse_exports(text, origin="<text>"):
    """Extract capabilities from annotated C header text."""
    out = []
    for m in EXPORT_RE.finditer(text):
        cap, tier, thread = m.group("cap"), m.group("tier"), m.group("thread")
        where = "%s: %s" % (origin, cap or "<empty>")

        if not CAP_RE.match(cap):
            raise ExportError("%s: capability name rejected by manifest schema pattern" % where)
        if tier not in TIERS:
            raise ExportError("%s: unknown tier %r" % (where, tier))
        if thread not in THREADS:
            raise ExportError("%s: unknown thread contract %r" % (where, thread))

        decl = re.sub(r"\s+", " ", m.group("decl")).strip()
        if "(" not in decl:
            raise ExportError("%s: annotation is not followed by a function declaration" % where)
        head, _, params = decl.partition("(")
        params = params.rstrip().rstrip(")")

        hm = re.match(r"^(?P<ret>.*?)(?P<name>[A-Za-z_]\w*)$", head.strip())
        if not hm:
            raise ExportError("%s: cannot parse return type / function name from %r" % (where, head))
        func = hm.group("name")
        ret = re.sub(r"\s+", " ", hm.group("ret")).strip()

        # Free consistency check: the capability name and the C name are two
        # spellings of the same thing, so they must agree. Catches copy-paste
        # drift with no extra annotation to keep in sync.
        expected = "skai_" + cap.replace(".", "_")
        if func != expected:
            raise ExportError("%s: declares %s(), expected %s()" % (where, func, expected))
        if ret not in RET_TYPES:
            raise ExportError("%s: return type %r is outside the SDK type vocabulary" % (where, ret))

        args = []
        if params.strip() not in ("", "void"):
            for p in params.split(","):
                args.append(_split_param(p))

        # Trailing (char *out, uint32_t cap) is the bounded-string idiom; it
        # collapses to a string return in the scripting projection.
        returns_string = (
            len(args) >= 2
            and args[-2][0] == "char *"
            and args[-1][0] == "uint32_t"
        )
        value_args = args[:-2] if returns_string else args

        sig = ""
        ts_args = []
        for atype, aname in value_args:
            if atype not in ARG_TYPES:
                raise ExportError(
                    "%s: parameter %s has type %r outside the SDK type vocabulary"
                    % (where, aname, atype)
                )
            tag, ts = ARG_TYPES[atype]
            sig += tag
            ts_args.append("%s: %s" % (aname, ts))

        # Display format for declarative (Tier 0) rendering. Lives in the table
        # so a new capability's units arrive with it; a second table in the
        # renderer would be exactly the drift this generator exists to prevent.
        ui = m.group("ui")
        if ui is not None and sig:
            raise ExportError(
                "%s: a display format only makes sense for a no-argument "
                "capability -- declarative binds cannot pass arguments" % where
            )
        if ui is None:
            ui = "%s" if returns_string else "%d"

        out.append(
            {
                "name": cap,
                "func": func,
                "tier": tier,
                "thread": thread,
                "args": sig,
                "returns": "string" if returns_string else RET_TYPES[ret][1],
                "ret_tag": "S" if returns_string else RET_TYPES[ret][0],
                "ui": ui,
                "doc": doc_before(text, m.start()),
                "decl": decl,
                "ts_args": ts_args,
            }
        )
    return out


def render_inc(caps):
    lines = [
        "/* GENERATED by tools/sdk/gen_dispatch.py -- do not edit. */",
        "/* Define SKAI_DISPATCH(idx, cap, fn, tier, thread, ret, args, ui)",
        "   before including. Entries are sorted by cap name (bsearch-able). */",
        "",
    ]
    for i, c in enumerate(caps):
        lines.append(
            'SKAI_DISPATCH(%d, "%s", %s, %s, %s, \'%s\', "%s", "%s")'
            % (i, c["name"], c["func"], c["tier"], c["thread"], c["ret_tag"],
               c["args"], c["ui"])
        )
    lines += ["", "#define SKAI_DISPATCH_COUNT %d" % len(caps), ""]
    return "\n".join(lines)


def c_unescape(s):
    """Turn C source escapes into the characters they denote.

    Display formats are written as C escapes in the headers (\\xc2\\xb0 rather
    than a literal degree sign) because MSVC mangles non-ASCII string literals
    without /utf-8. That spelling is correct for the generated .inc, which is C
    again -- but the phone-side registry is JSON, where it would arrive as a
    literal backslash and render as garbage.
    """
    raw = re.sub(r"\\x([0-9a-fA-F]{2})",
                 lambda m: chr(int(m.group(1), 16)), s)
    try:
        return raw.encode("latin-1").decode("utf-8")
    except (UnicodeEncodeError, UnicodeDecodeError):
        return raw


def render_registry(caps, major, minor):
    return json.dumps(
        {
            "_generated_by": "tools/sdk/gen_dispatch.py",
            "skai": "%d.%d" % (major, minor),
            "capabilities": [
                dict({k: c[k] for k in ("name", "tier", "thread", "args", "returns")},
                     ui=c_unescape(c["ui"]))
                for c in caps
            ],
        },
        indent=2,
        sort_keys=False,
    ) + "\n"


def render_all_header(headers):
    """One header that pulls in every capability domain.

    Exists so skai_dispatch.c never has to be edited again: it needs a
    declaration for every function named in the generated table, and forgetting
    one is a build break at a line number in generated code. Adding a domain
    stays a one-file change.
    """
    lines = [
        "/* GENERATED by tools/sdk/gen_dispatch.py -- do not edit. */",
        "#ifndef SKAI_ALL_H",
        "#define SKAI_ALL_H",
        "",
    ]
    for h in headers:
        lines.append('#include "skai/%s"' % h)
    lines += ["", "#endif /* SKAI_ALL_H */", ""]
    return "\n".join(lines)


TIER_NOTE = {
    "SKAI_T1": ("T1", "granted automatically"),
    "SKAI_T2": ("T2", "user consents per item at install"),
    "SKAI_T2_MIC": ("T2-mic", "consent plus a system in-use indicator, foreground only"),
    "SKAI_T3": ("T3", "never granted to a self-signed app"),
}
THREAD_NOTE = {
    "SKAI_THREAD_ANY": ("any thread", ""),
    "SKAI_THREAD_LVGL": ("LVGL thread", "calls from elsewhere are refused"),
    "SKAI_THREAD_APP": ("app thread", "blocks on flash; never call it from the UI thread"),
}
TS_TYPE = {"number": "number", "boolean": "boolean", "string": "string", "void": "void"}


def esc(s):
    return (s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def js_signature(c):
    return "skai.%s(%s): %s%s" % (
        c["name"], ", ".join(c["ts_args"]),
        TS_TYPE.get(c["returns"], c["returns"]),
        " | null" if c["returns"] in ("number", "string") else "",
    )


def render_html(caps, modules, major, minor):
    """API reference, generated from the same annotations as everything else.

    Hand-written docs for a generated API go stale the first time someone adds
    a capability, which is the exact failure this whole toolchain exists to
    prevent. The prose comes from the header comments; nothing here is a second
    copy of anything.
    """
    by_ns = {}
    for c in caps:
        by_ns.setdefault(c["name"].split(".")[0], []).append(c)

    nav = "\n".join(
        '<li><a href="#%s">skai.%s</a></li>' % (ns, ns) for ns in sorted(by_ns)
    )

    body = []
    for ns in sorted(by_ns):
        mod_doc = modules.get(ns, "")
        body.append('<section id="%s">' % ns)
        body.append("<h2>skai.%s</h2>" % ns)
        if mod_doc:
            body.append('<p class="lede">%s</p>' % esc(mod_doc))
        for c in by_ns[ns]:
            tier, tier_note = TIER_NOTE.get(c["tier"], (c["tier"], ""))
            thread, thread_note = THREAD_NOTE.get(c["thread"], (c["thread"], ""))
            body.append('<article id="%s">' % c["name"])
            body.append("<h3>%s</h3>" % esc(c["name"].split(".", 1)[1]))
            body.append('<pre class="sig">%s</pre>' % esc(js_signature(c)))
            body.append(
                '<p class="meta">'
                '<span class="badge tier-%s" title="%s">%s</span>'
                '<span class="badge thread" title="%s">%s</span>'
                "</p>" % (tier.replace("-", ""), esc(tier_note), tier,
                          esc(thread_note), thread)
            )
            for para in (c["doc"].split("\n\n") if c["doc"] else []):
                body.append("<p>%s</p>" % esc(para))
            if c["ts_args"]:
                body.append("<table><thead><tr><th>parameter</th><th>type</th>"
                            "</tr></thead><tbody>")
                for a in c["ts_args"]:
                    nm, _, ty = a.partition(": ")
                    body.append("<tr><td><code>%s</code></td><td><code>%s</code>"
                                "</td></tr>" % (esc(nm), esc(ty)))
                body.append("</tbody></table>")
            body.append('<p class="c"><span>C</span> <code>%s;</code></p>'
                        % esc(c["decl"]))
            body.append("</article>")
        body.append("</section>")

    return HTML_SHELL % {
        "version": "%d.%d" % (major, minor),
        "count": len(caps),
        "nav": nav,
        "body": "\n".join(body),
    }


HTML_SHELL = """<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Skai SDK %(version)s &mdash; API Reference</title>
<style>
:root{--bg:#fff;--fg:#1a1a1a;--muted:#6b6b6b;--line:#e3e3e3;--accent:#0a84ff;
      --code-bg:#f6f7f9;--sig-bg:#f0f4f8}
@media (prefers-color-scheme:dark){
:root{--bg:#16181c;--fg:#e6e6e6;--muted:#9aa0a6;--line:#2b2f36;
      --code-bg:#1e2127;--sig-bg:#1b2530}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
     font:16px/1.6 -apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}
code,pre{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
.wrap{display:flex;align-items:flex-start;max-width:1180px;margin:0 auto}
nav{position:sticky;top:0;width:220px;flex:none;padding:32px 16px;
    max-height:100vh;overflow:auto}
nav h1{font-size:15px;margin:0 0 4px}
nav .ver{color:var(--muted);font-size:13px;margin:0 0 20px}
nav ul{list-style:none;margin:0;padding:0}
nav li{margin:2px 0}
nav a{color:var(--fg);text-decoration:none;font-size:14px;display:block;
      padding:4px 8px;border-radius:6px}
nav a:hover{background:var(--code-bg);color:var(--accent)}
main{flex:1;min-width:0;padding:32px 24px 96px;border-left:1px solid var(--line)}
h2{font-size:26px;margin:48px 0 4px;padding-bottom:8px;border-bottom:1px solid var(--line)}
section:first-child h2{margin-top:0}
h3{font-size:18px;margin:0 0 8px}
article{padding:20px 0;border-bottom:1px solid var(--line)}
article:last-child{border-bottom:0}
p{margin:10px 0}
.lede{color:var(--muted)}
pre.sig{background:var(--sig-bg);border:1px solid var(--line);border-radius:8px;
        padding:10px 14px;overflow-x:auto;font-size:14px;margin:0 0 10px}
.meta{margin:0 0 12px}
.badge{display:inline-block;font-size:12px;padding:2px 9px;border-radius:999px;
       border:1px solid var(--line);margin-right:6px;color:var(--muted);cursor:help}
.tierT1{border-color:#2ea043;color:#2ea043}
.tierT2,.tierT2mic{border-color:#d29922;color:#d29922}
.tierT3{border-color:#f85149;color:#f85149}
table{border-collapse:collapse;margin:12px 0;font-size:14px;width:100%%;
      display:block;overflow-x:auto}
th,td{text-align:left;padding:6px 12px;border-bottom:1px solid var(--line)}
th{color:var(--muted);font-weight:600}
code{background:var(--code-bg);padding:1px 5px;border-radius:4px;font-size:13px}
.c{color:var(--muted);font-size:13px}
.c span{display:inline-block;border:1px solid var(--line);border-radius:4px;
        padding:0 5px;margin-right:4px;font-size:11px}
.note{background:var(--code-bg);border-left:3px solid var(--accent);
      padding:12px 16px;border-radius:0 8px 8px 0;margin:16px 0}
@media (max-width:760px){.wrap{display:block}nav{position:static;width:auto;
  max-height:none}main{border-left:0;padding:0 20px 64px}}
</style>
</head><body>
<div class="wrap">
<nav>
  <h1>Skai SDK</h1>
  <p class="ver">%(version)s &middot; %(count)d capabilities</p>
  <ul>%(nav)s</ul>
</nav>
<main>
<h2 style="margin-top:0">Overview</h2>
<p>Everything a watch app can do, as JavaScript. Each entry below is a
<em>capability</em>: your app declares the ones it needs in its manifest, and the
runtime refuses any call you did not declare.</p>
<div class="note">
<p><strong>Nothing is granted by default.</strong> A capability absent from your
manifest throws when called, and you can catch it. Use
<code>skai.available(name)</code> to branch instead of throwing &mdash; a
capability may be missing because the firmware is older or because the user
declined it.</p>
</div>
<p>Numeric readings return <code>null</code> when there is no measurement
(no heart-rate sample yet, no forecast received). That is a value you can test,
not a sentinel number to memorise.</p>
<p class="lede">Generated from the annotated SDK headers by
<code>tools/sdk/gen_dispatch.py</code>. Do not edit by hand.</p>
%(body)s
</main>
</div>
</body></html>
"""


def render_dts(caps, major, minor):
    lines = [
        "// GENERATED by tools/sdk/gen_dispatch.py -- do not edit.",
        "// Skai SDK %d.%d" % (major, minor),
        "",
        "declare namespace skai {",
        "  /** False when the firmware lacks this capability, or the manifest did not declare it. */",
        "  function available(capability: string): boolean;",
        "",
    ]
    by_ns = {}
    for c in caps:
        by_ns.setdefault(c["name"].split(".")[0], []).append(c)
    for ns in sorted(by_ns):
        lines.append("  namespace %s {" % ns)
        for c in by_ns[ns]:
            member = c["name"].split(".", 1)[1]
            lines.append(
                "    function %s(%s): %s;" % (member, ", ".join(c["ts_args"]), c["returns"])
            )
        lines += ["  }", ""]
    lines += ["}", ""]
    return "\n".join(lines)


def read_version(header):
    text = header.read_text(encoding="utf-8")
    def get(name):
        m = re.search(r"#define\s+%s\s+(\d+)" % name, text)
        if not m:
            raise ExportError("%s: %s not found" % (header, name))
        return int(m.group(1))
    return get("SKAI_API_MAJOR"), get("SKAI_API_MINOR")


SELFTEST_OK = """
SKAI_EXPORT("battery.level", SKAI_T1, SKAI_THREAD_ANY, "%d%%")
int32_t skai_battery_level(void);

SKAI_EXPORT("time.format", SKAI_T1, SKAI_THREAD_ANY)
int32_t skai_time_format(const char *fmt, char *out, uint32_t cap);
"""

SELFTEST_BAD = [
    # unknown type must be rejected, never silently dropped
    ('SKAI_EXPORT("x.y", SKAI_T1, SKAI_THREAD_ANY)\nint32_t skai_x_y(double d);', "vocabulary"),
    # C name must match the capability name
    ('SKAI_EXPORT("x.y", SKAI_T1, SKAI_THREAD_ANY)\nint32_t skai_x_z(void);', "expected"),
    # tier typo must not fall through to a permissive default
    ('SKAI_EXPORT("x.y", SKAI_T0, SKAI_THREAD_ANY)\nint32_t skai_x_y(void);', "tier"),
    # capability name must satisfy the manifest schema
    ('SKAI_EXPORT("XY", SKAI_T1, SKAI_THREAD_ANY)\nint32_t skai_XY(void);', "schema"),
    # a display format on a capability taking arguments is meaningless
    ('SKAI_EXPORT("x.y", SKAI_T1, SKAI_THREAD_ANY, "%d")\nint32_t skai_x_y(uint32_t a);',
     "no-argument"),
]


def selftest():
    caps = parse_exports(SELFTEST_OK, "selftest")
    assert len(caps) == 2, caps
    lvl, fmt = caps
    assert (lvl["name"], lvl["args"], lvl["returns"]) == ("battery.level", "", "number"), lvl
    assert lvl["ui"] == "%d%%", lvl        # explicit hint survives
    assert fmt["ui"] == "%s", fmt          # string capability defaults to %s
    # the (char *out, uint32_t cap) pair must collapse, not leak into the JS signature
    assert fmt["args"] == "S", fmt
    assert fmt["returns"] == "string", fmt
    assert fmt["ts_args"] == ["fmt: string"], fmt

    for src, needle in SELFTEST_BAD:
        try:
            parse_exports(src, "selftest")
        except ExportError as e:
            assert needle in str(e), "wrong error for %r: %s" % (needle, e)
        else:
            raise AssertionError("expected rejection: %r" % needle)

    # C escapes stay verbatim in the .inc (it is C again) but must be real
    # characters in the JSON the phone reads.
    assert c_unescape("%d\\xc2\\xb0") == "%d°", c_unescape("%d\\xc2\\xb0")
    assert c_unescape("%d%%") == "%d%%"

    assert "SKAI_DISPATCH_COUNT 2" in render_inc(caps)
    assert '"tier": "SKAI_T1"' in render_registry(caps, 1, 0)
    assert "function format(fmt: string): string;" in render_dts(caps, 1, 0)
    print("selftest ok")


def main():
    here = Path(__file__).resolve()
    default_src = here.parents[2] / "example/get-started/dualcore/src/modules/sdk/includes/skai"

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--src", type=Path, default=default_src)
    ap.add_argument("--out-dir", type=Path, default=None)
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return 0

    src = args.src
    out_dir = args.out_dir or src.parents[1] / "generated"
    major, minor = read_version(src / "skai_sdk_version.h")

    caps = []
    domain_headers = []
    modules = {}
    for header in sorted(src.glob("skai_*.h")):
        if header.name in ("skai_export.h", "skai_sdk_version.h",
                           "skai_dispatch.h", "skai_js.h"):
            continue
        text = header.read_text(encoding="utf-8")
        found = parse_exports(text, header.name)
        if found:
            domain_headers.append(header.name)
        caps += found
    # ponytail: no module blurbs. The headers' opening comments are written for
    # firmware engineers -- they name backing globals, unlinked components and
    # phase numbers -- and this page is read by third-party developers. The
    # per-capability comments are already externally worded, so they carry the
    # reference on their own. Give a domain its own outward-facing sentence here
    # if one is ever worth writing; do not pipe the internal one through.

    seen = {}
    for c in caps:
        if c["name"] in seen:
            raise ExportError("duplicate capability %s" % c["name"])
        seen[c["name"]] = c
    caps.sort(key=lambda c: c["name"])

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "skai_dispatch_table.inc").write_text(render_inc(caps), encoding="utf-8")
    (out_dir / "capability-registry.json").write_text(
        render_registry(caps, major, minor), encoding="utf-8"
    )
    (out_dir / "skai.d.ts").write_text(render_dts(caps, major, minor), encoding="utf-8")
    (out_dir / "skai_all.h").write_text(render_all_header(domain_headers), encoding="utf-8")
    (out_dir / "skai-api.html").write_text(
        render_html(caps, modules, major, minor), encoding="utf-8")

    print("skai %d.%d: %d capabilities -> %s" % (major, minor, len(caps), out_dir))
    for c in caps:
        print("  %-24s %-8s %s(%s) -> %s" % (c["name"], c["tier"], c["func"], c["args"], c["returns"]))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except ExportError as e:
        print("gen_dispatch: %s" % e, file=sys.stderr)
        sys.exit(1)
