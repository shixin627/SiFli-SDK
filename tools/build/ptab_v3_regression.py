#!/usr/bin/env python3
# SPDX-FileCopyrightText 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
ptab v3 regression (offline, strict numeric compare)

This script compares key artifacts produced by legacy ptab.json (v1/v2) build
artifacts vs regenerated outputs from migrated ptab.yaml (v3):
- ptab.h (numeric macro values)
- link_copy.lds (numeric values of __ROM_BASE/__RAM_BASE/... assignments)
- ftab.bin (partition entries: base/size/xip_base/flags)

It does NOT require running full SCons builds (use prebuilt baseline artifacts).

Example:
  python3 tools/build/ptab_v3_regression.py --board sf32lb52-nano_52j --chip SF32LB52JUD6
"""

import argparse
import ast
import os
import re
import struct
import sys
import tempfile
import types
from typing import Any, Dict, List, Optional, Tuple


SEC_CONFIG_MAGIC = 0x53454346  # 'FCES'
PARTITION_ENTRY_COUNT = 16
PARTITION_ENTRY_STRUCT = struct.Struct('<IIII')


def _repo_root() -> str:
    sifli = os.getenv('SIFLI_SDK')
    if sifli:
        return os.path.abspath(sifli)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _first_existing_path(candidates: List[str]) -> str:
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[0]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--board', default='sf32lb52-nano_52j', help='Board name (without _hcpu suffix)')
    parser.add_argument('--chip', default='SF32LB52JUD6', help='Chip part number used for migrated ptab v3')
    parser.add_argument('--ptab-a', default=None, help='First v3 ptab.yaml for direct semantic compare')
    parser.add_argument('--ptab-b', default=None, help='Second v3 ptab.yaml for direct semantic compare')
    parser.add_argument(
        '--baseline-build-dir',
        default=None,
        help='Baseline HCPU build dir (default: example/misc/button/project/build_<board>_hcpu)',
    )
    parser.add_argument('--skip-link', action='store_true', help='Skip link_copy.lds comparison')
    parser.add_argument('--keep-tmp', action='store_true', help='Keep generated files (print tmp dir)')
    return parser.parse_args()


class _EvalError(RuntimeError):
    pass


def _safe_eval_int(expr: str) -> int:
    expr = (expr or '').strip()
    if not expr:
        raise _EvalError('Empty expression')

    # Strip trailing semicolons and outer parentheses
    expr = expr.rstrip(';').strip()
    while expr.startswith('(') and expr.endswith(')'):
        inner = expr[1:-1].strip()
        if not inner:
            break
        expr = inner

    tree = ast.parse(expr, mode='eval')

    def _eval(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        if isinstance(node, ast.Constant):
            if isinstance(node.value, bool):
                return int(node.value)
            if isinstance(node.value, int):
                return node.value
            raise _EvalError('Unsupported constant: {}'.format(type(node.value)))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            v = _eval(node.operand)
            return +v if isinstance(node.op, ast.UAdd) else -v
        if isinstance(node, ast.BinOp):
            left = _eval(node.left)
            right = _eval(node.right)
            op = node.op
            if isinstance(op, ast.Add):
                return left + right
            if isinstance(op, ast.Sub):
                return left - right
            if isinstance(op, ast.Mult):
                return left * right
            if isinstance(op, ast.Div):
                return int(left / right)
            if isinstance(op, ast.FloorDiv):
                return left // right
            if isinstance(op, ast.Mod):
                return left % right
            if isinstance(op, ast.LShift):
                return left << right
            if isinstance(op, ast.RShift):
                return left >> right
            if isinstance(op, ast.BitOr):
                return left | right
            if isinstance(op, ast.BitAnd):
                return left & right
            if isinstance(op, ast.BitXor):
                return left ^ right
            raise _EvalError('Unsupported op: {}'.format(type(op)))
        raise _EvalError('Unsupported node: {}'.format(type(node)))

    return int(_eval(tree))


def _parse_c_defines(path: str) -> Dict[str, str]:
    define_re = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s+(.*)$')
    out: Dict[str, str] = {}
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = define_re.match(line)
            if not m:
                continue
            name = m.group(1)
            expr = (m.group(2) or '').strip()
            expr = expr.split('//', 1)[0].strip()
            out[name] = expr
    return out


def _eval_define(name: str, defs: Dict[str, str], cache: Dict[str, int]) -> int:
    if name in cache:
        return cache[name]
    if name not in defs:
        raise _EvalError('Unknown macro: {}'.format(name))
    expr = defs[name]

    # Simple reference case
    ref_match = re.fullmatch(r'\(?\s*([A-Za-z_]\w*)\s*\)?', expr)
    if ref_match and ref_match.group(1) in defs:
        v = _eval_define(ref_match.group(1), defs, cache)
        cache[name] = v
        return v

    # Replace identifiers with evaluated values
    tokens = set(re.findall(r'\b[A-Za-z_]\w*\b', expr))
    expanded = expr
    for tok in sorted(tokens, key=len, reverse=True):
        if tok not in defs:
            continue
        v = _eval_define(tok, defs, cache)
        expanded = re.sub(r'\b{}\b'.format(re.escape(tok)), str(v), expanded)

    v2 = _safe_eval_int(expanded)
    cache[name] = v2
    return v2


def _collect_macro_values(path: str) -> Dict[str, int]:
    defs = _parse_c_defines(path)
    cache: Dict[str, int] = {}
    out: Dict[str, int] = {}
    for k in defs.keys():
        try:
            out[k] = _eval_define(k, defs, cache)
        except Exception:
            # Skip non-numeric macros
            continue
    return out


def _collect_fal_part_entries(path: str) -> List[Tuple[str, str, int, int]]:
    entry_re = re.compile(
        r'\{FAL_PART_MAGIC_WORD,\s*"([^"]+)",\s*([A-Z0-9_]+),\s*0x([0-9A-Fa-f]+),\s*0x([0-9A-Fa-f]+),\s*0\}'
    )
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        text = f.read()
    out: List[Tuple[str, str, int, int]] = []
    for name, dev_name, offset_hex, size_hex in entry_re.findall(text):
        out.append((name, dev_name, int(offset_hex, 16), int(size_hex, 16)))
    return out


def _expected_fal_part_entries(ptab_obj) -> List[Tuple[str, str, int, int]]:
    root = _repo_root()
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    import ptab as ptab_module  # type: ignore

    def _is_auto_fal_partition(partition: Dict[str, Any]) -> bool:
        ptype = str(partition.get('type') or '').strip()
        subtype = str(partition.get('subtype') or '').strip()
        if ptype == 'data':
            return subtype in ('flashdb_kv', 'filesystem')
        if ptype == 'app':
            return subtype != 'ex'
        return False

    def _fal_device_macro_for_region(region: str) -> Optional[str]:
        region = str(region or '').strip().lower()
        match = re.match(r'^mpi([1-5])$', region)
        if match:
            return 'NOR_FLASH{}_DEV_NAME'.format(match.group(1))
        if region == 'sdmmc1':
            return 'SDMMC1_DEV_NAME'
        if region == 'sdmmc2':
            return 'SDMMC2_DEV_NAME'
        return None

    out: List[Tuple[str, str, int, int]] = []
    for partition in getattr(ptab_obj, 'partitions', []) or []:
        if not isinstance(partition, dict) or not _is_auto_fal_partition(partition):
            continue
        name = str(partition.get('name') or '').strip()
        if not name:
            continue
        dev_name = _fal_device_macro_for_region(str(partition.get('region') or '').strip())
        if not dev_name:
            raise _EvalError("partition '{}' uses unsupported auto-FAL region '{}'".format(
                name, partition.get('region')
            ))
        offset = ptab_module.parse_size(partition.get('offset', 0))
        size = ptab_module.parse_size(partition.get('size', 0))
        out.append((name, dev_name, offset, size))
    return out


def _check_generated_fal_part_table(ptab_obj, header_path: str, label: str) -> List[Tuple[str, str, int, int]]:
    actual = sorted(_collect_fal_part_entries(header_path))
    expected = sorted(_expected_fal_part_entries(ptab_obj))
    if actual != expected:
        raise _EvalError('{} FAL_PART_TABLE mismatch: {} != {}'.format(label, actual, expected))
    return actual


def _parse_lds_assignments(path: str) -> Dict[str, int]:
    assign_re = re.compile(r'^\s*(__[A-Za-z0-9_]+)\s*=\s*(.+?);')
    out: Dict[str, int] = {}
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = assign_re.match(line)
            if not m:
                # stop early after header constants
                if out and line.strip().startswith('MEMORY'):
                    break
                continue
            name = m.group(1)
            expr = m.group(2).strip()
            try:
                out[name] = _safe_eval_int(expr)
            except Exception:
                continue
    return out


def _parse_ftab_partitions(path: str) -> List[Tuple[int, int, int, int]]:
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 4 + PARTITION_ENTRY_STRUCT.size * PARTITION_ENTRY_COUNT:
        raise _EvalError('ftab.bin too small: {}'.format(len(data)))
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != SEC_CONFIG_MAGIC:
        raise _EvalError('Invalid ftab magic: 0x{:08X}'.format(magic))
    off = 4
    out: List[Tuple[int, int, int, int]] = []
    for _ in range(PARTITION_ENTRY_COUNT):
        base, size, xip_base, flags = PARTITION_ENTRY_STRUCT.unpack_from(data, off)
        out.append((base, size, xip_base, flags))
        off += PARTITION_ENTRY_STRUCT.size
    return out


def _write_generated_ptab_h(ptab_obj, env_name: str, core: str, out_path: str) -> None:
    root = _repo_root()
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    # Inject rtconfig for GenPartitionTableHeaderContentV3
    rt = types.ModuleType('rtconfig')
    rt.CORE = core
    sys.modules['rtconfig'] = rt

    import sdk_resource  # type: ignore

    sdk_resource.InitIndentation()
    env = {'name': env_name}
    s = ''
    s += sdk_resource.MakeLine('#ifndef __PTAB__H__')
    s += sdk_resource.MakeLine('#define __PTAB__H__')
    s += sdk_resource.MakeLine('')
    s += sdk_resource.MakeLine('')
    s += sdk_resource.GenPartitionTableHeaderContentV3(env, ptab_obj)
    s += sdk_resource.MakeLine('')
    s += sdk_resource.MakeLine('')
    s += sdk_resource.MakeLine('')
    s += sdk_resource.MakeLine('#endif')
    with open(out_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(s)


def _compare_ptab_h(root: str, ptab_obj_a, ptab_obj_b, tmp_dir: str) -> None:
    main_a = os.path.join(tmp_dir, 'ptab_a_main.h')
    main_b = os.path.join(tmp_dir, 'ptab_b_main.h')
    boot_a = os.path.join(tmp_dir, 'ptab_a_boot.h')
    boot_b = os.path.join(tmp_dir, 'ptab_b_boot.h')

    _write_generated_ptab_h(ptab_obj_a, 'main', 'HCPU', main_a)
    _write_generated_ptab_h(ptab_obj_b, 'main', 'HCPU', main_b)
    _write_generated_ptab_h(ptab_obj_a, 'bootloader', 'HCPU', boot_a)
    _write_generated_ptab_h(ptab_obj_b, 'bootloader', 'HCPU', boot_b)

    a_main = _collect_macro_values(main_a)
    b_main = _collect_macro_values(main_b)
    a_boot = _collect_macro_values(boot_a)
    b_boot = _collect_macro_values(boot_b)

    key_macros = [
        'FLASH_TABLE_START_ADDR', 'FLASH_TABLE_SIZE', 'FLASH_TABLE_OFFSET',
        'HCPU_FLASH_CODE_START_ADDR', 'HCPU_FLASH_CODE_SIZE', 'HCPU_FLASH_CODE_OFFSET',
        'DFU_FLASH_CODE_START_ADDR', 'DFU_FLASH_CODE_SIZE', 'DFU_FLASH_CODE_OFFSET',
        'DFU_DOWNLOAD_REGION_START_ADDR', 'DFU_DOWNLOAD_REGION_SIZE', 'DFU_DOWNLOAD_REGION_OFFSET',
        'FS_REGION_START_ADDR', 'FS_REGION_SIZE', 'FS_REGION_OFFSET',
        'KVDB_DFU_REGION_START_ADDR', 'KVDB_DFU_REGION_SIZE', 'KVDB_DFU_REGION_OFFSET',
        'KVDB_BLE_REGION_START_ADDR', 'KVDB_BLE_REGION_SIZE', 'KVDB_BLE_REGION_OFFSET',
        'PSRAM_DATA_START_ADDR', 'PSRAM_DATA_SIZE', 'PSRAM_DATA_OFFSET',
        'FLASH_BOOT_LOADER_START_ADDR', 'FLASH_BOOT_LOADER_SIZE', 'FLASH_BOOT_LOADER_OFFSET',
        'BOOTLOADER_RAM_DATA_START_ADDR', 'BOOTLOADER_RAM_DATA_SIZE', 'BOOTLOADER_RAM_DATA_OFFSET',
        'CODE_START_ADDR', 'CODE_SIZE',
    ]

    def _check(label: str, left: Dict[str, int], right: Dict[str, int]) -> None:
        for key in key_macros:
            if key not in left and key not in right:
                continue
            if key not in left or key not in right:
                raise _EvalError('{} macro presence mismatch: {}'.format(label, key))
            if int(left[key]) != int(right[key]):
                raise _EvalError(
                    '{} macro mismatch {}: 0x{:08X} != 0x{:08X}'.format(
                        label, key, int(left[key]) & 0xFFFFFFFF, int(right[key]) & 0xFFFFFFFF
                    )
                )

    _check('main', a_main, b_main)
    _check('bootloader', a_boot, b_boot)

    fal_main_a = _check_generated_fal_part_table(ptab_obj_a, main_a, 'main-a')
    fal_main_b = _check_generated_fal_part_table(ptab_obj_b, main_b, 'main-b')
    if fal_main_a != fal_main_b:
        raise _EvalError('main FAL_PART_TABLE mismatch: {} != {}'.format(fal_main_a, fal_main_b))


def _compare_ftab(ptab_obj_a, ptab_obj_b) -> None:
    root = _repo_root()
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    import gen_ftab  # type: ignore

    a_blob = gen_ftab.generate_ftab_binary(ptab_obj_a, ptab_obj_a.get_chip_config(), 0x10000, 0x200000)
    b_blob = gen_ftab.generate_ftab_binary(ptab_obj_b, ptab_obj_b.get_chip_config(), 0x10000, 0x200000)
    a_parts = _parse_ftab_partitions_bytes(a_blob)
    b_parts = _parse_ftab_partitions_bytes(b_blob)
    for idx in [0, 1, 3, 4, 7, 8]:
        if a_parts[idx] != b_parts[idx]:
            raise _EvalError('ftab entry {} mismatch: {} != {}'.format(idx, a_parts[idx], b_parts[idx]))


def _parse_ftab_partitions_bytes(data: bytes) -> List[Tuple[int, int, int, int]]:
    if len(data) < 4 + PARTITION_ENTRY_STRUCT.size * PARTITION_ENTRY_COUNT:
        raise _EvalError('ftab.bin too small: {}'.format(len(data)))
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != SEC_CONFIG_MAGIC:
        raise _EvalError('Invalid ftab magic: 0x{:08X}'.format(magic))
    off = 4
    out: List[Tuple[int, int, int, int]] = []
    for _ in range(PARTITION_ENTRY_COUNT):
        base, size, xip_base, flags = PARTITION_ENTRY_STRUCT.unpack_from(data, off)
        out.append((base, size, xip_base, flags))
        off += PARTITION_ENTRY_STRUCT.size
    return out


def main() -> int:
    args = _parse_args()
    root = _repo_root()

    if bool(args.ptab_a) != bool(args.ptab_b):
        print('ERROR: --ptab-a and --ptab-b must be provided together', file=sys.stderr)
        return 2

    baseline_dir = args.baseline_build_dir
    if not baseline_dir:
        baseline_dir = os.path.join(root, 'example', 'misc', 'button', 'project', 'build_{}_hcpu'.format(args.board))
    baseline_dir = os.path.abspath(baseline_dir)

    baseline_main_ptab_h = os.path.join(baseline_dir, 'ptab.h')
    baseline_main_link = os.path.join(baseline_dir, 'link_copy.lds')
    baseline_ftab = os.path.join(baseline_dir, 'ftab', 'ftab.bin')
    baseline_main_bin = _first_existing_path([
        os.path.join(baseline_dir, 'output', 'main.app.bin'),
        os.path.join(baseline_dir, 'output', 'main.bin'),
        os.path.join(baseline_dir, 'main.bin'),
    ])

    baseline_boot_dir = os.path.join(baseline_dir, 'bootloader')
    baseline_boot_ptab_h = os.path.join(baseline_boot_dir, 'ptab.h')
    baseline_boot_link = os.path.join(baseline_boot_dir, 'link_copy.lds')
    baseline_boot_bin = _first_existing_path([
        os.path.join(baseline_boot_dir, 'output', 'bootloader.app.bin'),
        os.path.join(baseline_boot_dir, 'output', 'bootloader.bin'),
        os.path.join(baseline_boot_dir, 'bootloader.bin'),
    ])

    # Prepare python imports from tools/build
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    import ptab as ptab_module  # type: ignore
    import migrate_ptab_to_v3  # type: ignore

    if args.ptab_a and args.ptab_b:
        tmp_dir = tempfile.mkdtemp(prefix='ptab_v3_reg_')
        try:
            ptab_a = ptab_module.load_ptab(args.ptab_a, fatal=True)
            ptab_b = ptab_module.load_ptab(args.ptab_b, fatal=True)
            if not ptab_a.is_v3() or not ptab_b.is_v3():
                raise _EvalError('direct compare mode only supports v3 ptab.yaml files')
            _compare_ptab_h(root, ptab_a, ptab_b, tmp_dir)
            _compare_ftab(ptab_a, ptab_b)
            if args.keep_tmp:
                print('OK (tmp dir kept): {}'.format(tmp_dir))
            else:
                try:
                    import shutil
                    shutil.rmtree(tmp_dir)
                except Exception:
                    pass
                print('OK')
            return 0
        except Exception:
            if not args.keep_tmp:
                try:
                    import shutil
                    shutil.rmtree(tmp_dir)
                except Exception:
                    pass
            raise

    ptab_json = os.path.join(root, 'customer', 'boards', args.board, 'ptab.json')
    if not os.path.exists(ptab_json):
        print('ERROR: baseline ptab.json not found: {}'.format(ptab_json), file=sys.stderr)
        return 2

    import gen_ftab  # type: ignore
    gen_link_lds = None
    if not args.skip_link:
        try:
            import gen_link_lds as _gen_link_lds  # type: ignore
            gen_link_lds = _gen_link_lds
        except Exception as exc:
            print('WARNING: skip link compare: {}'.format(exc), file=sys.stderr)

    # Migrate to v3 YAML into tmp dir
    tmp_dir = tempfile.mkdtemp(prefix='ptab_v3_reg_')
    v3_yaml_path = os.path.join(tmp_dir, 'ptab.yaml')
    v3_dict = migrate_ptab_to_v3.migrate_ptab_json_to_v3(migrate_ptab_to_v3._load_ptab_json(ptab_json), args.chip)
    with open(v3_yaml_path, 'w', encoding='utf-8', newline='\n') as f:
        yaml_text = yaml_dump(v3_dict)
        f.write(yaml_text)

    ptab_obj = ptab_module.load_ptab(v3_yaml_path, fatal=True)

    # Generate v3 ptab.h (main + bootloader)
    gen_main_ptab_h = os.path.join(tmp_dir, 'ptab_main.h')
    gen_boot_ptab_h = os.path.join(tmp_dir, 'ptab_boot.h')
    _write_generated_ptab_h(ptab_obj, 'main', 'HCPU', gen_main_ptab_h)
    _write_generated_ptab_h(ptab_obj, 'bootloader', 'HCPU', gen_boot_ptab_h)
    _check_generated_fal_part_table(ptab_obj, gen_main_ptab_h, 'generated main')

    # Compare ptab.h (numeric)
    baseline_main_macros = _collect_macro_values(baseline_main_ptab_h)
    gen_main_macros = _collect_macro_values(gen_main_ptab_h)
    baseline_boot_macros = _collect_macro_values(baseline_boot_ptab_h)
    gen_boot_macros = _collect_macro_values(gen_boot_ptab_h)

    key_macros = [
        'FLASH_TABLE_START_ADDR', 'FLASH_TABLE_SIZE', 'FLASH_TABLE_OFFSET',
        'HCPU_FLASH_CODE_START_ADDR', 'HCPU_FLASH_CODE_SIZE', 'HCPU_FLASH_CODE_OFFSET',
        'DFU_FLASH_CODE_START_ADDR', 'DFU_FLASH_CODE_SIZE', 'DFU_FLASH_CODE_OFFSET',
        'DFU_DOWNLOAD_REGION_START_ADDR', 'DFU_DOWNLOAD_REGION_SIZE', 'DFU_DOWNLOAD_REGION_OFFSET',
        'FS_REGION_START_ADDR', 'FS_REGION_SIZE', 'FS_REGION_OFFSET',
        'KVDB_DFU_REGION_START_ADDR', 'KVDB_DFU_REGION_SIZE', 'KVDB_DFU_REGION_OFFSET',
        'KVDB_BLE_REGION_START_ADDR', 'KVDB_BLE_REGION_SIZE', 'KVDB_BLE_REGION_OFFSET',
        'PSRAM_DATA_START_ADDR', 'PSRAM_DATA_SIZE', 'PSRAM_DATA_OFFSET',
        'FLASH_BOOT_LOADER_START_ADDR', 'FLASH_BOOT_LOADER_SIZE', 'FLASH_BOOT_LOADER_OFFSET',
        'BOOTLOADER_RAM_DATA_START_ADDR', 'BOOTLOADER_RAM_DATA_SIZE', 'BOOTLOADER_RAM_DATA_OFFSET',
        'CODE_START_ADDR', 'CODE_SIZE',
    ]

    def _check_macros(label: str, base: Dict[str, int], new: Dict[str, int]) -> None:
        for k in key_macros:
            if k not in base:
                continue
            if k not in new:
                raise _EvalError('{} missing macro: {}'.format(label, k))
            if int(base[k]) != int(new[k]):
                raise _EvalError('{} macro mismatch {}: 0x{:08X} != 0x{:08X}'.format(label, k, int(base[k]) & 0xFFFFFFFF, int(new[k]) & 0xFFFFFFFF))

    _check_macros('main', baseline_main_macros, gen_main_macros)
    _check_macros('bootloader', baseline_boot_macros, gen_boot_macros)

    if gen_link_lds is not None:
        # Generate v3 link_copy.lds (main + bootloader)
        hcpu_template = os.path.join(root, 'drivers', 'cmsis', 'sf32lb52x', 'Templates', 'gcc', 'hcpu', 'link.jinja2')
        boot_template = os.path.join(root, 'example', 'boot_loader', 'project', 'butterflmicro', 'ram_v2', 'link.jinja2')

        main_rtconfig_h = os.path.join(baseline_dir, 'rtconfig.h')
        boot_rtconfig_h = os.path.join(baseline_boot_dir, 'rtconfig.h')
        main_defines = gen_link_lds._parse_rtconfig_defines(main_rtconfig_h)
        boot_defines = gen_link_lds._parse_rtconfig_defines(boot_rtconfig_h)

        main_link_out = os.path.join(tmp_dir, 'link_main.lds')
        boot_link_out = os.path.join(tmp_dir, 'link_boot.lds')

        gen_link_lds.render_link_lds(
            hcpu_template,
            main_link_out,
            gen_link_lds.compute_link_defines(ptab_obj, 'main', 'HCPU', main_defines),
        )
        gen_link_lds.render_link_lds(
            boot_template,
            boot_link_out,
            gen_link_lds.compute_link_defines(ptab_obj, 'bootloader', 'HCPU', boot_defines),
        )

        lds_keys = [
            '__ROM_BASE', '__ROM_SIZE', '__RAM_BASE', '__RAM_SIZE',
            '__ROM_EX_BASE', '__ROM_EX_SIZE', '__PSRAM_BASE', '__PSRAM_SIZE',
            '__ROM2_BASE', '__ROM2_SIZE', '__ROM3_BASE', '__ROM3_SIZE',
        ]
        baseline_main_lds = _parse_lds_assignments(baseline_main_link)
        gen_main_lds = _parse_lds_assignments(main_link_out)
        for k in lds_keys:
            if k not in baseline_main_lds or k not in gen_main_lds:
                continue
            if int(baseline_main_lds[k]) != int(gen_main_lds[k]):
                raise _EvalError('main link mismatch {}: 0x{:08X} != 0x{:08X}'.format(k, int(baseline_main_lds[k]) & 0xFFFFFFFF, int(gen_main_lds[k]) & 0xFFFFFFFF))

        baseline_boot_lds = _parse_lds_assignments(baseline_boot_link)
        gen_boot_lds = _parse_lds_assignments(boot_link_out)
        for k in lds_keys:
            if k not in baseline_boot_lds or k not in gen_boot_lds:
                continue
            if int(baseline_boot_lds[k]) != int(gen_boot_lds[k]):
                raise _EvalError('bootloader link mismatch {}: 0x{:08X} != 0x{:08X}'.format(k, int(baseline_boot_lds[k]) & 0xFFFFFFFF, int(gen_boot_lds[k]) & 0xFFFFFFFF))

    # Generate v3 ftab.bin (main)
    boot_size = os.path.getsize(baseline_boot_bin) if os.path.exists(baseline_boot_bin) else 0x10000
    main_size = os.path.getsize(baseline_main_bin) if os.path.exists(baseline_main_bin) else 0x200000
    chip_config = ptab_obj.get_chip_config()
    ftab_data = gen_ftab.generate_ftab_binary(ptab_obj, chip_config, boot_size, main_size)
    gen_ftab_path = os.path.join(tmp_dir, 'ftab.bin')
    with open(gen_ftab_path, 'wb') as f:
        f.write(ftab_data)

    base_ftab_parts = _parse_ftab_partitions(baseline_ftab)
    gen_ftab_parts = _parse_ftab_partitions(gen_ftab_path)

    # Compare key partition entries (base/size/xip_base/flags)
    key_indices = [0, 1, 3, 4, 7, 8]
    for idx in key_indices:
        if base_ftab_parts[idx] != gen_ftab_parts[idx]:
            raise _EvalError('ftab entry {} mismatch: {} != {}'.format(idx, base_ftab_parts[idx], gen_ftab_parts[idx]))

    if args.keep_tmp:
        print('OK (tmp dir kept): {}'.format(tmp_dir))
    else:
        # Best-effort cleanup
        try:
            import shutil
            shutil.rmtree(tmp_dir)
        except Exception:
            pass
        print('OK')
    return 0


def yaml_dump(obj: Any) -> str:
    # Local import to avoid hard dependency when running under minimal envs.
    import yaml  # type: ignore
    from collections import OrderedDict

    class _OrderedDumper(yaml.SafeDumper):
        pass

    def _repr_ordered(dumper: yaml.Dumper, data: Dict[str, Any]):  # type: ignore
        return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())

    _OrderedDumper.add_representer(dict, _repr_ordered)
    _OrderedDumper.add_representer(OrderedDict, _repr_ordered)
    return yaml.dump(obj, Dumper=_OrderedDumper, default_flow_style=False, allow_unicode=True, sort_keys=False, indent=2)


if __name__ == '__main__':
    raise SystemExit(main())
