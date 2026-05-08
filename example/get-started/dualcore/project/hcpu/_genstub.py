import re, os, glob, sys

SDK_DIRS = [
    r'C:\work\SiFli-SDK\example\get-started\dualcore\src',
    r'C:\work\SiFli-SDK\middleware\include',
    r'C:\work\SiFli-SDK\middleware\system',
    r'C:\work\SiFli-SDK\middleware\data_bus\public',
]

syms = []
with open(r'C:\work\SiFli-SDK\example\get-started\dualcore\project\hcpu\_syms_clean.txt') as f:
    for line in f:
        line = line.strip()
        if line.startswith('_'):
            syms.append(line[1:])

def find_decl(sym):
    for d in SDK_DIRS:
        for path in glob.glob(os.path.join(d, '**', '*.h'), recursive=True) + glob.glob(os.path.join(d, '**', '*.c'), recursive=True):
            try:
                with open(path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
            except Exception:
                continue
            pattern = rf'(?:^|[\s;}}])((?:extern\s+)?[\w][\w\s\*]*?)\b{re.escape(sym)}\s*\(([^;{{)]*)\)\s*[;{{]'
            for m in re.finditer(pattern, content, re.MULTILINE):
                ret = m.group(1).strip()
                args = m.group(2).strip()
                if ret.startswith('extern'):
                    ret = ret[len('extern'):].strip()
                if ret in ('', 'static', 'inline'):
                    continue
                if not re.match(r'^[\w][\w\s\*]*$', ret):
                    continue
                if 'sizeof' in ret or 'return' in ret:
                    continue
                return (ret, args, path)
    return None

stubs = []
unfound = []

for sym in syms:
    decl = find_decl(sym)
    if not decl:
        unfound.append(sym)
        continue
    ret, args, src = decl
    ret_trim = ret.replace('*', '').strip()
    if 'void' == ret_trim:
        body = ''
    elif ret_trim in ('bool',):
        body = 'return false;'
    elif '*' in ret:
        body = 'return 0;'
    else:
        body = 'return 0;'
    args_norm = 'void' if not args else args
    stubs.append((sym, ret, args_norm, body, os.path.basename(src)))

with open(r'C:\work\SiFli-SDK\example\get-started\dualcore\src\hcpu\pc_link_stubs.c', 'w', encoding='utf-8') as out:
    out.write('/* Auto-generated stubs for PC simulator (do not edit by hand).\n')
    out.write(f' * Covers {len(stubs)} symbols normally provided by ARM-only modules\n')
    out.write(' * (BLE stack, voice/skai/gesture apps, IPC peripherals, etc.).\n')
    out.write(' * Each function logs once and returns a sensible default. */\n\n')
    out.write('#include <rtthread.h>\n')
    out.write('#include <rtdevice.h>\n')
    out.write('#include <stdbool.h>\n')
    out.write('#include <stdint.h>\n\n')
    for sym, ret, args, body, src in stubs:
        out.write(f'/* {src} */ {ret} {sym}({args}) {{ {body} }}\n')
    out.write('\n/* Unresolved (no declaration found):\n')
    for s in unfound:
        out.write(f' *   {s}\n')
    out.write(' */\n')

print(f'Generated {len(stubs)} stubs. Unfound: {len(unfound)}')
for s in unfound:
    print(f'  - {s}')
