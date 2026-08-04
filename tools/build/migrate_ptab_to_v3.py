#!/usr/bin/env python3
# SPDX-FileCopyrightText 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Migrate legacy ptab.json (v1/v2) to ptab.yaml (v3).

This tool focuses on keeping build compatibility:
- Converts regions to v3 `partitions` with `type/subtype/region/offset/size`.
- Converts exec-only regions (region has `exec` but no `img`) into `exec` on the
  corresponding `bootloader/app` partition.
- Uses legacy `tags` as `aliases` (dedup, case-insensitive) when needed.

Usage:
  python3 tools/build/migrate_ptab_to_v3.py -i customer/boards/<board>/ptab.json -o /tmp/ptab.yaml -c SF32LB52JUD6
"""

import argparse
import copy
import os
import json
import re
import sys
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple

import yaml

import ptab as ptab_module


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Migrate ptab.json (v1/v2) to ptab.yaml (v3)',
        allow_abbrev=False,
    )
    parser.add_argument('-i', '--input', required=True, help='Input ptab.json path')
    parser.add_argument('-o', '--output', help='Output ptab.yaml path (default: stdout)')
    parser.add_argument(
        '-c', '--chip',
        default='SF32LB52JUD6',
        help='Chip part number for ptab v3 (default: SF32LB52JUD6)',
    )
    parser.add_argument('--no-header', action='store_true', help='Do not emit header comments')
    return parser.parse_args()


def _load_ptab_json(path: str) -> List[Dict[str, Any]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    if not isinstance(data, list):
        raise ValueError('Invalid ptab.json: expected a list')
    return data


def _content_mems(ptab_json: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if ptab_json and isinstance(ptab_json[0], dict) and 'version' in ptab_json[0]:
        return copy.deepcopy(ptab_json[1:])
    return copy.deepcopy(ptab_json)


def _chip_series(chip: str) -> str:
    m = re.match(r'^(SF32LB\d{2})', str(chip or '').strip(), flags=re.IGNORECASE)
    return m.group(1).upper() if m else ''


def _ensure_default_regions(ptab_json: List[Dict[str, Any]], chip: str) -> List[Dict[str, Any]]:
    mems = _content_mems(ptab_json)
    series = _chip_series(chip)

    if series in ('SF32LB56', 'SF32LB58'):
        ftab_found = False
        bootloader_found = False
        flash5_mem: Optional[Dict[str, Any]] = None

        for mem in mems:
            if str(mem.get('mem') or '').strip().lower() == 'flash5':
                flash5_mem = mem
            for region in mem.get('regions') or []:
                name = str(region.get('name') or '').strip().lower()
                tags_upper = [str(t).strip().upper() for t in (region.get('tags') or []) if str(t).strip()]
                ftab_name = str(((region.get('ftab') or {}).get('name') or '')).strip().lower() if isinstance(region.get('ftab'), dict) else ''
                img = str(region.get('img') or '').strip().lower()
                if name == 'ftab' or ftab_name == 'ftab' or img == 'ftab' or 'FLASH_TABLE' in tags_upper:
                    ftab_found = True
                if name == 'bootloader' or ftab_name == 'bootloader' or img == 'bootloader' or 'FLASH_BOOT_LOADER' in tags_upper:
                    bootloader_found = True
            if ftab_found and bootloader_found:
                break

        if not flash5_mem:
            flash5_mem = OrderedDict([
                ('mem', 'flash5'),
                ('base', '0x1C000000'),
                ('regions', []),
            ])
            mems.insert(0, flash5_mem)

        regions = flash5_mem.setdefault('regions', [])
        if not ftab_found:
            regions.insert(0, OrderedDict([
                ('offset', '0x00000000'),
                ('max_size', '0x00005000' if series == 'SF32LB58' else '0x00008000'),
                ('tags', ['FLASH_TABLE']),
                ('name', 'ftab'),
                ('type', ['app_img', 'app_exec']),
            ]))
        if not bootloader_found:
            regions.insert(1 if not ftab_found else 0, OrderedDict([
                ('offset', '0x00020000'),
                ('max_size', '0x00020000'),
                ('tags', ['FLASH_BOOT_LOADER']),
                ('name', 'bootloader'),
                ('type', ['app_img', 'app_exec']),
            ]))

    return mems


def _mem_to_region(mem_name: str) -> str:
    mem_name = (mem_name or '').strip().lower()
    mapping = {
        'flash1': 'mpi1',
        'flash2': 'mpi2',
        'flash3': 'mpi3',
        'flash4': 'mpi4',
        'flash5': 'mpi5',
        'psram1': 'psram1',
        'psram1_cbus': 'psram1',
        'psram2': 'psram2',
        'hpsys_ram': 'hpsys_ram',
        'lpsys_ram': 'lpsys_ram',
    }
    return mapping.get(mem_name, mem_name)


def _board_name_from_input(input_path: str) -> str:
    try:
        return os.path.basename(os.path.dirname(os.path.abspath(input_path)))
    except Exception:
        return ''


def _infer_external_memory(board_name: str, mem: Dict[str, Any], chip: str) -> Optional[Dict[str, Any]]:
    mem_name = str(mem.get('mem') or '').strip().lower()
    region = _mem_to_region(mem_name)
    if not region.startswith('mpi'):
        return None

    series = _chip_series(chip)
    if series in ('SF32LB56', 'SF32LB58') and region == 'mpi5':
        # Boot flash on 56/58 is chip-internal and should not be described in
        # board-level `memory`.
        return None

    base = str(mem.get('base') or '').strip().lower()
    m = re.search(r'[_-]([an])(\d+)r', board_name.lower())
    if m:
        mtype = 'nand' if m.group(1) == 'a' else 'nor'
        size = int(m.group(2)) * 1024 * 1024
        return OrderedDict([
            ('mpi', region),
            ('type', mtype),
            ('size', size),
        ])

    if mem_name == 'flash4':
        return OrderedDict([
            ('mpi', region),
            ('type', 'nor'),
        ])
    if mem_name == 'flash2':
        return OrderedDict([
            ('mpi', region),
            ('type', 'nand' if base == '0x62000000' else 'nor'),
        ])
    return None


def _sanitize_name(name: str) -> str:
    name = (name or '').strip().lower()
    name = re.sub(r'[^a-z0-9_]', '_', name)
    if not name:
        return name
    if name[0].isdigit():
        name = 'p_' + name
    return name


def _dedup_aliases(aliases: List[str], canonical_name: str) -> List[str]:
    seen = set()
    out: List[str] = []
    canonical_upper = (canonical_name or '').upper()
    for a in aliases or []:
        a = str(a or '').strip()
        if not a:
            continue
        up = a.upper()
        if up == canonical_upper:
            continue
        if up in seen:
            continue
        seen.add(up)
        out.append(a)
    return out


def _type_list(region_data: Dict[str, Any]) -> List[str]:
    out: List[str] = []
    types = region_data.get('type')
    if isinstance(types, list):
        for t in types:
            t = str(t or '').strip().lower()
            if t:
                out.append(t)
    return out


def _ftab_name(region_data: Dict[str, Any]) -> str:
    ftab = region_data.get('ftab') if isinstance(region_data.get('ftab'), dict) else {}
    return str(ftab.get('name') or '').strip().lower()


def _region_name_hint(region_data: Dict[str, Any]) -> str:
    for value in (region_data.get('name'), region_data.get('img'), _ftab_name(region_data)):
        value = str(value or '').strip().lower()
        if value:
            return value
    return ''


def _has_ftab_xip_only(region_data: Dict[str, Any]) -> bool:
    ftab = region_data.get('ftab') if isinstance(region_data.get('ftab'), dict) else {}
    addrs = ftab.get('address') or []
    if not isinstance(addrs, list):
        return False
    addrs_norm = [str(a).strip().lower() for a in addrs if str(a).strip()]
    return bool(addrs_norm) and 'xip' in addrs_norm and 'base' not in addrs_norm


def _keep_exec_descriptor_as_partition(region_data: Dict[str, Any]) -> bool:
    tags_upper = [str(t).strip().upper() for t in (region_data.get('tags') or []) if str(t).strip()]
    return 'ACPU_CODE_REGION1_SBUS' in tags_upper


def _infer_type_subtype_core(region: str, region_data: Dict[str, Any]) -> Tuple[str, Optional[str], Optional[str]]:
    tags = region_data.get('tags') or []
    tags = [str(t) for t in tags if str(t).strip()]
    img = str(region_data.get('img') or '').strip().lower()
    ftab_name = _ftab_name(region_data)
    name_hint = _region_name_hint(region_data)
    type_list = _type_list(region_data)
    core_hint = str(region_data.get('core') or '').strip().upper() or None

    tags_upper = [t.upper() for t in tags]

    # ftab
    if 'FLASH_TABLE' in tags_upper or img == 'ftab' or ftab_name == 'ftab' or name_hint == 'ftab':
        return 'ftab', None, None

    # bootloader
    if img == 'bootloader' or ftab_name == 'bootloader' or name_hint == 'bootloader' or 'FLASH_BOOT_LOADER' in tags_upper:
        return 'bootloader', None, 'HCPU'

    # ACPU app
    if (
        name_hint in ('acpu', 'acpu_region1')
        or any(t.startswith('ACPU_CODE_LOAD_REGION') for t in tags_upper)
    ) and ('app_img' in type_list or img or ftab_name):
        return 'app', 'factory', 'ACPU'

    # app (factory/dfu)
    if 'DFU_FLASH_CODE' in tags_upper or img == 'dfu' or ftab_name == 'dfu':
        return 'app', 'dfu', 'HCPU'
    if (
        'HCPU_FLASH_CODE' in tags_upper
        or 'HCPU_FLASH_CODE_LOAD_REGION' in tags_upper
        or img == 'main'
        or ftab_name == 'main'
        or name_hint == 'main'
        or ('app_img' in type_list and name_hint == 'main')
    ):
        return 'app', 'factory', 'HCPU'

    # filesystem / KVDB
    if 'FS_REGION' in tags_upper:
        return 'data', 'filesystem', None
    if any(t.startswith('KVDB_') for t in tags_upper):
        return 'data', 'flashdb_kv', None

    # RAM-like regions
    if region in ('hpsys_ram', 'lpsys_ram') or region.startswith('psram'):
        return 'data', 'ram', None

    # default
    return 'data', 'raw', core_hint


def _infer_partition_name(region_data: Dict[str, Any], ptype: str, subtype: Optional[str], core: Optional[str]) -> str:
    tags = region_data.get('tags') or []
    tags_upper = [str(t).strip().upper() for t in tags if str(t).strip()]
    img = str(region_data.get('img') or '').strip().lower()
    ftab_name = _ftab_name(region_data)
    name_hint = _region_name_hint(region_data)

    if ptype == 'ftab':
        return 'flash_table'
    if ptype == 'bootloader':
        return 'bootloader'
    if ptype == 'app':
        if (core or '').upper() == 'ACPU':
            return 'acpu'
        if subtype == 'dfu':
            return 'dfu_flash_code'
        if subtype == 'factory':
            if any(t in ('HCPU_FLASH_CODE', 'HCPU_FLASH_CODE_LOAD_REGION') for t in tags_upper):
                return 'hcpu_flash_code'
            if name_hint == 'main' or img == 'main' or ftab_name == 'main':
                return 'hcpu_flash_code'
            return _sanitize_name(name_hint or 'app')

    tags = [str(t) for t in tags if str(t).strip()]

    # FlashDB KV: use DB/FAL partition name from tag `KVDB_<NAME>_REGION`.
    for t in tags:
        m = re.match(r'^KVDB_(.+)_REGION$', str(t).strip(), flags=re.IGNORECASE)
        if m:
            return _sanitize_name(m.group(1))

    # Common legacy tag mappings
    preferred_tag_names = {
        'DFU_INFO_REGION': 'dfu_info_region',
        'DFU_DOWNLOAD_REGION': 'dfu_download_region',
        'BOOTLOADER_RAM_DATA': 'bootloader_ram_data',
        'HCPU_RAM_DATA': 'hcpu_ram_data',
        'HCPU_RO_DATA': 'hcpu_ro_data',
        'LPSYS_RAM': 'lpsys_ram',
        'PSRAM_DATA': 'psram_data',
        'PSRAM_DATA2': 'psram_data2',
        'FS_EX': 'fs_ex_region',
        'ACPU_CMD_BUF': 'acpu_cmd_buf',
        'ACPU_CODE_REGION1_SBUS': 'acpu_code_region1_sbus',
        'ACPU_CODE_LOAD_REGION1': 'acpu',
    }
    for t in tags_upper:
        if t in preferred_tag_names:
            return preferred_tag_names[t]

    if ptype == 'data' and subtype == 'filesystem':
        return 'fs_region'

    # Prefer first tag as it maps to legacy macro base names.
    if tags:
        return _sanitize_name(tags[0])
    if name_hint:
        return _sanitize_name(name_hint)
    if ftab_name:
        return _sanitize_name(ftab_name)
    if img:
        return _sanitize_name(img)
    return ''


def migrate_ptab_json_to_v3(ptab_json: List[Dict[str, Any]], chip: str, input_path: Optional[str] = None) -> Dict[str, Any]:
    # Map exec-only regions: img_name -> {region, offset}
    exec_only: Dict[str, Dict[str, Any]] = {}

    # Collect all regions first
    regions_flat: List[Tuple[str, Dict[str, Any]]] = []
    mems = _ensure_default_regions(ptab_json, chip)
    for mem in mems:
        if not isinstance(mem, dict):
            continue
        region = _mem_to_region(mem.get('mem'))
        for r in mem.get('regions') or []:
            if not isinstance(r, dict):
                continue
            regions_flat.append((region, r))

            exec_name = str(r.get('exec') or '').strip()
            img_name = str(r.get('img') or '').strip()
            name_hint = _region_name_hint(r)
            type_list = _type_list(r)
            if exec_name and not img_name and 'app_img' not in type_list:
                # exec-only region (exec source)
                exec_only[exec_name.lower()] = {
                    'region': region,
                    'offset': r.get('offset', '0'),
                }
                continue

            if 'app_exec' in type_list and 'app_img' not in type_list and name_hint:
                exec_only.setdefault(name_hint.lower(), {
                    'region': region,
                    'offset': r.get('offset', '0'),
                })
                continue

            if _has_ftab_xip_only(r) and not img_name and name_hint:
                # Legacy v1 format often described XIP / load-to-RAM destinations as
                # standalone regions with only `ftab.address: [xip]`. They provide
                # execution metadata, not a separate partition.
                exec_only.setdefault(name_hint.lower(), {
                    'region': region,
                    'offset': r.get('offset', '0'),
                })

    partitions: List[Dict[str, Any]] = []

    for region, r in regions_flat:
        type_list = _type_list(r)
        name_hint = _region_name_hint(r)

        # Skip exec-only regions; they'll be converted into exec
        if r.get('exec') and not r.get('img') and 'app_img' not in type_list:
            continue

        # Skip pure v2 app_exec or v1 xip-descriptor regions; they only describe
        # execution placement for a code partition stored elsewhere.
        if (
            (('app_exec' in type_list and 'app_img' not in type_list) or (_has_ftab_xip_only(r) and not r.get('img')))
            and not _keep_exec_descriptor_as_partition(r)
        ):
            continue

        # Skip zero-sized regions
        try:
            if int(str(r.get('max_size', '0')), 0) == 0:
                continue
        except Exception:
            pass

        # Legacy ptab.json may contain overlapping "macro regions" in internal RAM.
        # For v3 we only keep non-overlapping partitions by default.
        tags = r.get('tags') or []
        tags_upper = [str(t).strip().upper() for t in tags if str(t).strip()]
        if region in ('hpsys_ram', 'lpsys_ram'):
            if any(t in (
                'HCPU_RAM_DATA',
                'HCPU_RO_DATA',
                'HPSYS_MBOX',
                'HCPU2LCPU_MB_CH2_BUF',
                'HCPU2LCPU_MB_CH1_BUF',
                'LCPU2HCPU_MB_CH1_BUF',
                'LCPU2HCPU_MB_CH2_BUF',
            ) for t in tags_upper):
                # Keep the non-overlapping RAM windows that are still consumed by
                # legacy link/header generation.
                if (
                    'BOOTLOADER_RAM_DATA' not in tags_upper
                    and 'HCPU_RAM_DATA' not in tags_upper
                    and 'LPSYS_RAM' not in tags_upper
                ):
                    continue

        ptype, subtype, core = _infer_type_subtype_core(region, r)
        name = _infer_partition_name(r, ptype, subtype, core)
        if not name:
            continue

        offset = r.get('offset', '0')
        size = r.get('max_size', '0')

        # Build aliases from legacy tags
        tags = [str(t) for t in tags if str(t).strip()]
        if ptype == 'data' and subtype == 'flashdb_kv':
            # v3 generates KVDB_*_REGION_* macros from partition name, so do not
            # carry legacy KVDB tag as an alias macro base.
            tags = [t for t in tags if not re.match(r'^KVDB_.+_REGION$', str(t).strip(), flags=re.IGNORECASE)]
        if ptype == 'data' and subtype == 'filesystem':
            tags = [t for t in tags if str(t).strip().upper() != 'FS_REGION']
        if name == 'acpu_code_region1_sbus':
            tags = [t for t in tags if str(t).strip().upper() != 'ACPU_CODE_REGION1_SBUS']
            tags.append('ACPU_CODE_REGION1')
        aliases = _dedup_aliases(tags, name)

        alias_upper = {str(a).strip().upper() for a in aliases}
        legacy_flash2_kind = None
        if name == 'hcpu_flash2_img' or 'HCPU_FLASH2_IMG' in alias_upper:
            legacy_flash2_kind = 'img'
        elif name == 'hcpu_flash2_font' or 'HCPU_FLASH2_FONT' in alias_upper:
            legacy_flash2_kind = 'font'

        if legacy_flash2_kind is not None:
            ptype = 'app'
            subtype = 'ex'
            core = core or 'HCPU'

        part: Dict[str, Any] = OrderedDict()
        part['name'] = name
        part['type'] = ptype
        if subtype:
            part['subtype'] = subtype
        part['region'] = region
        part['offset'] = offset
        part['size'] = size
        if core:
            part['core'] = core
        if aliases:
            part['aliases'] = aliases
        if legacy_flash2_kind == 'font':
            part['sections'] = [
                OrderedDict([
                    ('object', 'lvsf_font_*'),
                    ('section', '.rodata*'),
                ])
            ]

        # Convert exec-only region into exec for bootloader/app partitions
        if ptype in ('bootloader', 'app') and subtype != 'ex':
            # Determine image name to search for exec-only mapping.
            exec_key = (str(r.get('img') or '').strip() or name_hint or _ftab_name(r)).lower()
            if exec_key and exec_key in exec_only:
                exec_src = exec_only[exec_key]
                part['exec'] = OrderedDict([
                    ('region', exec_src.get('region', '')),
                    ('offset', exec_src.get('offset', '0')),
                ])
            elif 'app_exec' in type_list:
                # V2 default regions (e.g. 58-series bootloader on flash5) may encode
                # img+exec in the same legacy region.
                part['exec'] = OrderedDict([
                    ('region', region),
                    ('offset', offset),
                ])

        partitions.append(part)

    # Auto-add calibration partition for common layouts (compat with legacy ftab)
    has_calibration = False
    flash_table_part: Optional[Dict[str, Any]] = None
    for p in partitions:
        if p.get('type') == 'ftab':
            flash_table_part = p
        if p.get('type') == 'data' and p.get('subtype') == 'calibration':
            has_calibration = True

    if flash_table_part and not has_calibration:
        try:
            flash_off = int(str(flash_table_part.get('offset', '0')), 0)
        except Exception:
            flash_off = 0
        try:
            flash_size = int(str(flash_table_part.get('size', '0')), 0)
        except Exception:
            flash_size = 0x8000
        cal_off = flash_off + flash_size
        cal_part: Dict[str, Any] = OrderedDict()
        cal_part['name'] = 'calibration'
        cal_part['type'] = 'data'
        cal_part['subtype'] = 'calibration'
        cal_part['region'] = flash_table_part.get('region', 'mpi2')
        cal_part['offset'] = '0x{:08X}'.format(cal_off)
        cal_part['size'] = '0x{:08X}'.format(0x2000)
        # Insert right after flash table for readability
        idx = partitions.index(flash_table_part) + 1
        partitions.insert(idx, cal_part)

    memory: List[Dict[str, Any]] = []
    board_name = _board_name_from_input(input_path or '')
    seen_regions = set()
    for mem in mems:
        entry = _infer_external_memory(board_name, mem, chip)
        if not entry:
            continue
        region = str(entry.get('mpi') or '').strip().lower()
        if not region or region in seen_regions:
            continue
        seen_regions.add(region)
        memory.append(entry)

    out: Dict[str, Any] = OrderedDict()
    out['version'] = 3
    out['chip'] = chip
    if memory:
        out['memory'] = memory
    out['partitions'] = partitions
    return ptab_module.prune_default_partitions_v3_data(out)


class _OrderedDumper(yaml.SafeDumper):
    pass


def _ordered_dict_representer(dumper: yaml.Dumper, data: OrderedDict) -> yaml.nodes.MappingNode:
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())


_OrderedDumper.add_representer(OrderedDict, _ordered_dict_representer)


def main() -> int:
    args = _parse_args()

    ptab_json = _load_ptab_json(args.input)
    v3 = migrate_ptab_json_to_v3(ptab_json, args.chip, input_path=args.input)

    yaml_body = yaml.dump(
        v3,
        Dumper=_OrderedDumper,
        default_flow_style=False,
        allow_unicode=True,
        sort_keys=False,
        indent=2,
    )

    if not args.no_header:
        header = (
            '# ptab v3 - Partition Table\n'
            '# Migrated from: {}\n'
            '# Chip: {}\n'
            '#\n'
            '# Auto-generated. Please review and adjust as needed.\n\n'
        ).format(os.path.basename(args.input), args.chip)
        yaml_body = header + yaml_body

    if args.output:
        with open(args.output, 'w', encoding='utf-8', newline='\n') as f:
            f.write(yaml_body)
        print('Written to: {}'.format(args.output))
    else:
        sys.stdout.write(yaml_body)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
