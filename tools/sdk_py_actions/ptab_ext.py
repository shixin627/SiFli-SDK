# SPDX-FileCopyrightText: 2025-2026 SiFli Technologies Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import math
import os
import sys
import tomllib
from typing import Any, Dict, List, Optional, Tuple

import click
import yaml

from sdk_py_actions.cli.context import SdkContext
from sdk_py_actions.cli.registry import CommandRegistry
from sdk_py_actions.errors import FatalError

EXTENSION_ID = "ptab"
EXTENSION_VERSION = "2.1.0"
EXTENSION_API_VERSION = 2
MIN_SDK_VERSION = None

_PTAB_USAGE_SCHEMA_VERSION = 1


def _read_project_config(project_dir: str) -> Dict[str, Optional[str]]:
    config_path = os.path.join(project_dir, '.project.toml')
    config = {
        'board': None,
        'board_search_path': None,
    }
    if not os.path.exists(config_path):
        return config

    try:
        with open(config_path, 'rb') as f:
            data = tomllib.load(f)
    except Exception as e:
        raise FatalError(f'Failed to read .project.toml: {e}')

    config['board'] = data.get('board')
    config['board_search_path'] = data.get('board_search_path')
    return config


def _resolve_board_options(
    project_dir: str,
    board: Optional[str],
    board_search_path: Optional[str],
) -> Tuple[str, Optional[str]]:
    if not board or board_search_path is None:
        config = _read_project_config(project_dir)
        if not board:
            board = config.get('board')
        if board_search_path is None:
            board_search_path = config.get('board_search_path')

    if not board:
        raise FatalError('Board name is required. Use "--board" or run "sdk.py set-target" first.')

    if board_search_path:
        board_search_path = os.path.abspath(board_search_path)

    return board, board_search_path


def _get_sdk_root(sdk_ctx: SdkContext) -> str:
    env = getattr(sdk_ctx, 'env', {}) or {}
    sdk_root = env.get('SIFLI_SDK_PATH') or os.environ.get('SIFLI_SDK_PATH')
    if not sdk_root:
        raise FatalError('SIFLI_SDK_PATH is not set. Run export.sh before using sdk.py ptab.')
    return os.path.abspath(sdk_root)


def _load_ptab_module(sdk_root: str):
    build_tools = os.path.join(sdk_root, 'tools', 'build')
    if build_tools not in sys.path:
        sys.path.insert(0, build_tools)
    import ptab as ptab_module
    return ptab_module


def _resolve_ptab_context(
    sdk_ctx: SdkContext,
    board: Optional[str],
    board_search_path: Optional[str],
    strict: bool,
    command_name: str,
) -> Dict[str, Any]:
    sdk_root = _get_sdk_root(sdk_ctx)
    project_dir = os.path.realpath(sdk_ctx.project_dir)
    board, board_search_path = _resolve_board_options(project_dir, board, board_search_path)

    ptab_module = _load_ptab_module(sdk_root)
    board = ptab_module.normalize_board_name(board)
    chip = ptab_module.detect_chip_dir_from_board(
        board,
        board_search_path=board_search_path,
        sdk_root=sdk_root,
    )
    if not chip:
        raise FatalError(f'Unable to detect CHIP from board rtconfig.py: {board}')

    board_path, _ = ptab_module.get_board_paths(
        board,
        board_search_path=board_search_path,
        sdk_root=sdk_root,
    )

    prepared = ptab_module.prepare_project_ptab(
        project_dir,
        board,
        chip,
        board_path=board_path,
        emit_summary=False,
        validate=True,
        strict_validation=strict,
    )

    ptab_obj = prepared.get('ptab_obj')
    if not ptab_obj or not ptab_obj.is_v3():
        raise FatalError(f'{command_name} only supports ptab v3 projects.')

    return {
        'sdk_root': sdk_root,
        'project_dir': project_dir,
        'board': board,
        'board_search_path': board_search_path,
        'chip': chip,
        'board_path': board_path,
        'prepared': prepared,
        'ptab_obj': ptab_obj,
        'ptab_module': ptab_module,
        'chip_config': ptab_obj.get_chip_config(),
    }


def _validation_counts(report: Dict[str, Any]) -> Tuple[int, int]:
    issues = report.get('validation', []) or []
    error_count = sum(1 for issue in issues if getattr(issue, 'severity', '') == 'error')
    warning_count = sum(1 for issue in issues if getattr(issue, 'severity', '') == 'warning')
    return error_count, warning_count


def _emit_ptab_report(ptab_module: Any, prepared: Dict[str, Any]) -> None:
    report = prepared.get('report', {}) or {}
    ptab_module.print_overlay_summary(report, dedupe=False)
    error_count, warning_count = _validation_counts(report)
    click.echo(
        f'Validated PTAB v3: {error_count} error(s), {warning_count} warning(s)',
        err=True,
    )


def ptab_export_callback(
    sdk_ctx: SdkContext,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
    output_path: Optional[str] = None,
    strict: bool = False,
) -> None:
    context = _resolve_ptab_context(
        sdk_ctx,
        board,
        board_search_path,
        strict,
        'ptab export',
    )
    prepared = context['prepared']
    ptab_module = context['ptab_module']

    effective_data = prepared.get('merged_data')
    if effective_data is None:
        ptab_path = prepared.get('path')
        if not ptab_path:
            raise FatalError('Unable to determine PTAB path for export.')
        with open(ptab_path, encoding='utf-8-sig') as f:
            effective_data = yaml.safe_load(f)

    output_yaml = ptab_module.dump_ptab_yaml(effective_data)

    if output_path:
        output_path = os.path.abspath(output_path)
        ptab_module.write_effective_ptab_yaml(output_path, effective_data)
        prepared.get('report', {})['effective_path'] = output_path
        click.echo(f'Wrote effective PTAB YAML to {output_path}', err=True)
    else:
        prepared.get('report', {})['effective_path'] = '<stdout>'
        click.echo(output_yaml, nl=False)

    _emit_ptab_report(ptab_module, prepared)


def _format_size(size: Optional[int]) -> str:
    if size is None:
        return '-'
    if size == 0:
        return '0'
    sign = '-' if size < 0 else ''
    size = abs(int(size))
    if size >= 1024 * 1024 * 1024:
        return f'{sign}{size / (1024 * 1024 * 1024):.1f}GB'
    if size >= 1024 * 1024:
        return f'{sign}{size / (1024 * 1024):.1f}MB'
    if size >= 1024:
        return f'{sign}{size / 1024:.1f}KB'
    return f'{sign}{size}B'


def _hex_or_none(value: Optional[int], width: int = 8) -> Optional[str]:
    if value is None:
        return None
    return f'0x{int(value):0{width}X}'


def _region_to_mpi(region: str) -> Optional[str]:
    region = (region or '').strip().lower()
    if not region:
        return None
    if region.startswith('mpi'):
        return region
    if region == 'psram':
        return 'mpi1'
    if region.startswith('psram'):
        suffix = region[len('psram'):] or '1'
        return f'mpi{suffix}'
    return None


def _get_region_memory_type(region: str, chip_config: Dict[str, Any]) -> str:
    region = (region or '').strip().lower()
    if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
        return 'ram'
    if region.startswith('sdmmc'):
        info = (chip_config.get('memory_info') or {}).get(region, {})
        return str(info.get('type') or 'sd')

    mpi = _region_to_mpi(region)
    if mpi:
        info = (chip_config.get('memory_info') or {}).get(mpi, {})
        if region == 'psram' or region.startswith('psram'):
            return str(info.get('type') or 'psram')
        return str(info.get('type') or 'nor')

    return ''


def _canonical_usage_region(region: str, chip_config: Dict[str, Any]) -> str:
    region_s = (region or '').strip()
    region_l = region_s.lower()
    if region_l == 'psram':
        return 'psram1'
    if region_l.startswith('psram'):
        return region_l

    mpi = _region_to_mpi(region_l)
    if mpi and _get_region_memory_type(region_l, chip_config).lower() == 'psram':
        suffix = mpi[len('mpi'):] or '1'
        return f'psram{suffix}'

    return region_s


def _select_storage_address(memory_type: str, sbus_addr: int, cbus_addr: int) -> int:
    return cbus_addr if (memory_type or '').lower() == 'nor' else sbus_addr


def _select_exec_address(memory_type: str, sbus_addr: int, cbus_addr: int) -> int:
    return sbus_addr if (memory_type or '').lower() in ('ram', 'nand') else cbus_addr


def _resolve_entry_address(
    ptab_module: Any,
    region: str,
    offset: int,
    chip_config: Dict[str, Any],
    core: Optional[str],
    kind: str,
) -> int:
    sbus_addr, cbus_addr = ptab_module.resolve_region_address(
        region,
        offset,
        chip_config,
        core=core,
    )
    memory_type = _get_region_memory_type(region, chip_config)
    if kind == 'exec':
        return _select_exec_address(memory_type, sbus_addr, cbus_addr)
    return _select_storage_address(memory_type, sbus_addr, cbus_addr)


def _resolve_region_base(
    ptab_module: Any,
    region: str,
    chip_config: Dict[str, Any],
) -> Optional[int]:
    try:
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, 0, chip_config)
    except Exception:
        return None
    memory_type = _get_region_memory_type(region, chip_config)
    return _select_storage_address(memory_type, sbus_addr, cbus_addr)


def _ram_region_capacity(region: str, chip_config: Dict[str, Any]) -> Optional[int]:
    region = (region or '').strip().lower()
    if region == 'hpsys_ram' or region.startswith('hpsys'):
        group = 'hpsys'
    elif region == 'lpsys_ram' or region.startswith('lpsys'):
        group = 'lpsys'
    else:
        return None

    ram_banks = (((chip_config.get('ram') or {}).get(group) or {}).get('ram') or {})
    total = 0
    for bank in ram_banks.values():
        if not isinstance(bank, dict):
            continue
        bank_size = None
        for core_key in ('hcpu', 'lcpu', 'acpu'):
            core_def = bank.get(core_key)
            if isinstance(core_def, dict) and core_def.get('size') is not None:
                bank_size = int(core_def.get('size') or 0)
                break
        if bank_size:
            total += bank_size
    return total or None


def _region_capacity(region: str, chip_config: Dict[str, Any]) -> Optional[int]:
    region = (region or '').strip().lower()
    if not region:
        return None

    ram_size = _ram_region_capacity(region, chip_config)
    if ram_size is not None:
        return ram_size

    memory_info = chip_config.get('memory_info') or {}
    info_key = region
    mpi = _region_to_mpi(region)
    if mpi:
        info_key = mpi
    info = memory_info.get(info_key, {})
    if isinstance(info, dict) and info.get('size') is not None:
        size = int(info.get('size') or 0)
        if size > 0:
            return size

    if mpi:
        mpi_def = (chip_config.get('mpi') or {}).get(mpi, {})
        for view in ('xip', 'base'):
            view_def = mpi_def.get(view)
            if isinstance(view_def, dict) and view_def.get('size') is not None:
                size = int(view_def.get('size') or 0)
                if size > 0:
                    return size

    return None


def _make_usage_entry(
    ptab_module: Any,
    partition: Dict[str, Any],
    chip_config: Dict[str, Any],
    *,
    kind: str,
    region: str,
    source_region: str,
    offset: int,
    size: int,
) -> Dict[str, Any]:
    core = partition.get('core')
    memory_type = _get_region_memory_type(source_region, chip_config)
    address = _resolve_entry_address(
        ptab_module,
        source_region,
        offset,
        chip_config,
        core=core,
        kind=kind,
    )
    end_offset = offset + size
    return {
        'name': str(partition.get('name') or ''),
        'kind': kind,
        'region': region,
        'source_region': source_region,
        'type': str(partition.get('type') or ''),
        'subtype': partition.get('subtype'),
        'core': core,
        'offset': offset,
        'offset_hex': _hex_or_none(offset),
        'end_offset': end_offset,
        'end_offset_hex': _hex_or_none(end_offset),
        'size_bytes': size,
        'size_hex': _hex_or_none(size),
        'address': address,
        'address_hex': _hex_or_none(address),
        'memory_type': memory_type,
    }


def _merge_intervals(intervals: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
    normalized = sorted((max(0, int(start)), max(0, int(end))) for start, end in intervals if end > start)
    merged: List[Tuple[int, int]] = []
    for start, end in normalized:
        if not merged or start > merged[-1][1]:
            merged.append((start, end))
        else:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
    return merged


def _find_gaps(region: str, total: Optional[int], merged: List[Tuple[int, int]]) -> List[Dict[str, Any]]:
    if total is None or total <= 0:
        return []
    gaps: List[Dict[str, Any]] = []
    cursor = 0
    for start, end in merged:
        clipped_start = min(max(start, 0), total)
        clipped_end = min(max(end, 0), total)
        if clipped_start > cursor:
            gaps.append(_make_gap(region, cursor, clipped_start))
        cursor = max(cursor, clipped_end)
    if cursor < total:
        gaps.append(_make_gap(region, cursor, total))
    return gaps


def _make_gap(region: str, start: int, end: int) -> Dict[str, Any]:
    return {
        'region': region,
        'offset': start,
        'offset_hex': _hex_or_none(start),
        'end_offset': end,
        'end_offset_hex': _hex_or_none(end),
        'size_bytes': end - start,
        'size_hex': _hex_or_none(end - start),
    }


def _find_overlaps(entries: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    overlaps: List[Dict[str, Any]] = []
    by_region: Dict[str, List[Dict[str, Any]]] = {}
    for entry in entries:
        if entry.get('kind') != 'storage':
            continue
        if str(entry.get('memory_type') or '').lower() in ('ram', 'psram'):
            continue
        by_region.setdefault(entry['region'], []).append(entry)

    for region, region_entries in by_region.items():
        sorted_entries = sorted(region_entries, key=lambda item: (item['offset'], item['end_offset'], item['name']))
        for idx, left in enumerate(sorted_entries):
            for right in sorted_entries[idx + 1:]:
                if right['offset'] >= left['end_offset']:
                    break
                start = max(left['offset'], right['offset'])
                end = min(left['end_offset'], right['end_offset'])
                if start >= end:
                    continue
                overlaps.append({
                    'region': region,
                    'entries': [left['name'], right['name']],
                    'kinds': [left['kind'], right['kind']],
                    'offset': start,
                    'offset_hex': _hex_or_none(start),
                    'end_offset': end,
                    'end_offset_hex': _hex_or_none(end),
                    'size_bytes': end - start,
                    'size_hex': _hex_or_none(end - start),
                })
    return overlaps


def build_ptab_usage_data(
    ptab_module: Any,
    prepared: Dict[str, Any],
    ptab_obj: Any,
    chip_config: Dict[str, Any],
    project_dir: str,
    board: str,
    chip: str,
) -> Dict[str, Any]:
    entries: List[Dict[str, Any]] = []
    for partition in ptab_obj.partitions:
        if not isinstance(partition, dict):
            continue
        region = str(partition.get('region') or '').strip()
        if not region:
            continue
        usage_region = _canonical_usage_region(region, chip_config)
        offset = ptab_module.parse_size(partition.get('offset', 0))
        size = ptab_module.parse_size(partition.get('size', 0))
        if size <= 0:
            continue

        entries.append(
            _make_usage_entry(
                ptab_module,
                partition,
                chip_config,
                kind='storage',
                region=usage_region,
                source_region=region,
                offset=offset,
                size=size,
            )
        )

        exec_def = partition.get('exec')
        if isinstance(exec_def, dict):
            exec_region = str(exec_def.get('region') or '').strip()
            exec_usage_region = _canonical_usage_region(exec_region, chip_config)
            if exec_region and exec_usage_region != usage_region:
                exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
                entries.append(
                    _make_usage_entry(
                        ptab_module,
                        partition,
                        chip_config,
                        kind='exec',
                        region=exec_usage_region,
                        source_region=exec_region,
                        offset=exec_offset,
                        size=size,
                    )
                )

    entries.sort(key=lambda item: (item['region'], item['offset'], item['kind'], item['name']))
    overlaps = _find_overlaps(entries)

    regions: List[Dict[str, Any]] = []
    gaps: List[Dict[str, Any]] = []
    region_names = sorted({entry['region'] for entry in entries})
    for region in region_names:
        region_entries = [entry for entry in entries if entry['region'] == region]
        intervals = [(entry['offset'], entry['end_offset']) for entry in region_entries]
        merged = _merge_intervals(intervals)
        used_bytes = sum(end - start for start, end in merged)
        total_bytes = _region_capacity(region, chip_config)
        free_bytes = None
        usage_percent = None
        if total_bytes is not None:
            free_bytes = max(0, total_bytes - used_bytes)
            usage_percent = (used_bytes / total_bytes * 100) if total_bytes > 0 else None
        region_gaps = _find_gaps(region, total_bytes, merged)
        gaps.extend(region_gaps)

        base_address = _resolve_region_base(ptab_module, region, chip_config)
        regions.append({
            'name': region,
            'memory_type': _get_region_memory_type(region, chip_config),
            'base_address': base_address,
            'base_address_hex': _hex_or_none(base_address),
            'total_bytes': total_bytes,
            'total_hex': _hex_or_none(total_bytes),
            'used_bytes': used_bytes,
            'used_hex': _hex_or_none(used_bytes),
            'free_bytes': free_bytes,
            'free_hex': _hex_or_none(free_bytes),
            'usage_percent': usage_percent,
            'entry_count': len(region_entries),
            'overlap_count': sum(1 for overlap in overlaps if overlap['region'] == region),
        })

    regions.sort(key=lambda item: (
        item['base_address'] is None,
        item['base_address'] if item['base_address'] is not None else 0,
        item['name'],
    ))
    report = prepared.get('report', {}) or {}
    error_count, warning_count = _validation_counts(report)

    chip_name = getattr(ptab_obj, 'chip', None) or chip
    return {
        'schema_version': _PTAB_USAGE_SCHEMA_VERSION,
        'metadata': {
            'project_dir': project_dir,
            'board': board,
            'chip': chip_name,
            'chip_dir': chip,
            'base_path': prepared.get('base_path'),
            'effective_path': prepared.get('effective_path'),
            'uses_overlay': bool(prepared.get('uses_overlay')),
            'overlay_paths': prepared.get('overlay_paths') or {},
        },
        'regions': regions,
        'partitions': entries,
        'gaps': gaps,
        'overlaps': overlaps,
        'validation': {
            'errors': error_count,
            'warnings': warning_count,
        },
    }


def _format_percent(value: Optional[float]) -> str:
    if value is None:
        return '-'
    return f'{value:.1f}%'


def _build_region_bar(
    region: Dict[str, Any],
    entries: List[Dict[str, Any]],
    overlaps: List[Dict[str, Any]],
    width: int,
):
    from rich.text import Text

    total = region.get('total_bytes')
    if total is None:
        max_end = max((entry['end_offset'] for entry in entries), default=0)
        total = max_end or 1

    chars = ['.' for _ in range(width)]
    styles = ['dim' for _ in range(width)]

    def mark(start: int, end: int, char: str, style: str) -> None:
        if total <= 0:
            return
        col_start = max(0, min(width - 1, int(start / total * width)))
        col_end = max(col_start + 1, min(width, int(math.ceil(end / total * width))))
        for col in range(col_start, col_end):
            chars[col] = char
            styles[col] = style

    for entry in entries:
        if entry['kind'] == 'exec':
            mark(entry['offset'], entry['end_offset'], '=', 'blue')
        else:
            mark(entry['offset'], entry['end_offset'], '#', 'green')

    for overlap in overlaps:
        mark(overlap['offset'], overlap['end_offset'], '!', 'bold red')

    text = Text()
    for char, style in zip(chars, styles):
        text.append(char, style=style)
    return text


def _print_ptab_usage_overview(console: Any, usage_data: Dict[str, Any]) -> None:
    from rich.panel import Panel

    metadata = usage_data['metadata']
    validation = usage_data['validation']
    overlay_paths = metadata.get('overlay_paths') or {}
    lines = [
        f"[bold]Project[/bold]: {metadata.get('project_dir')}",
        f"[bold]Board[/bold]: {metadata.get('board')}",
        f"[bold]Chip[/bold]: {metadata.get('chip')}",
        f"[bold]Base PTAB[/bold]: {metadata.get('base_path')}",
        f"[bold]Effective PTAB[/bold]: {metadata.get('effective_path')}",
        f"[bold]Overlay[/bold]: {'yes' if metadata.get('uses_overlay') else 'no'}",
        f"[bold]Validation[/bold]: {validation.get('errors', 0)} error(s), {validation.get('warnings', 0)} warning(s)",
    ]
    if overlay_paths.get('chip'):
        lines.append(f"[bold]Chip Overlay[/bold]: {overlay_paths.get('chip')}")
    if overlay_paths.get('board'):
        lines.append(f"[bold]Board Overlay[/bold]: {overlay_paths.get('board')}")
    console.print(Panel.fit('\n'.join(lines), title='PTAB Usage'))


def print_ptab_usage_layout(usage_data: Dict[str, Any]) -> None:
    from rich.console import Console
    from rich.table import Table

    console = Console()
    _print_ptab_usage_overview(console, usage_data)
    console.print('Legend: [green]#[/green]=storage [blue]=[/blue]=exec [dim].[/dim]=free [bold red]![/bold red]=overlap')

    table = Table(title='Memory Layout', show_header=True, header_style='bold magenta')
    table.add_column('Region', style='cyan')
    table.add_column('Type', style='green')
    table.add_column('Usage', justify='right')
    table.add_column('Layout')

    bar_width = max(32, min(96, console.width - 45))
    for region in usage_data['regions']:
        entries = [entry for entry in usage_data['partitions'] if entry['region'] == region['name']]
        overlaps = [overlap for overlap in usage_data['overlaps'] if overlap['region'] == region['name']]
        if region.get('total_bytes') is not None:
            usage = '{} / {} ({})'.format(
                _format_size(region.get('used_bytes')),
                _format_size(region.get('total_bytes')),
                _format_percent(region.get('usage_percent')),
            )
        else:
            usage = '{} / -'.format(_format_size(region.get('used_bytes')))
        table.add_row(
            region['name'],
            region.get('memory_type') or '-',
            usage,
            _build_region_bar(region, entries, overlaps, bar_width),
        )

    console.print(table)
    print_ptab_usage_tables(usage_data, console=console, include_overview=False)


def print_ptab_usage_tables(
    usage_data: Dict[str, Any],
    console: Optional[Any] = None,
    include_overview: bool = True,
) -> None:
    from rich.console import Console
    from rich.table import Table

    console = console or Console()
    if include_overview:
        _print_ptab_usage_overview(console, usage_data)

    summary = Table(title='Region Summary', show_header=True, header_style='bold magenta')
    summary.add_column('Region', style='cyan')
    summary.add_column('Type', style='green')
    summary.add_column('Base', style='yellow', justify='right')
    summary.add_column('Total', justify='right')
    summary.add_column('Used', justify='right')
    summary.add_column('Free', justify='right')
    summary.add_column('Usage', justify='right')
    summary.add_column('Entries', justify='right')
    summary.add_column('Overlaps', justify='right')
    for region in usage_data['regions']:
        summary.add_row(
            region['name'],
            region.get('memory_type') or '-',
            region.get('base_address_hex') or '-',
            _format_size(region.get('total_bytes')),
            _format_size(region.get('used_bytes')),
            _format_size(region.get('free_bytes')),
            _format_percent(region.get('usage_percent')),
            str(region.get('entry_count') or 0),
            str(region.get('overlap_count') or 0),
        )
    console.print(summary)

    parts = Table(title='Partition Usage', show_header=True, header_style='bold magenta')
    parts.add_column('Region', style='cyan')
    parts.add_column('Kind', style='blue')
    parts.add_column('Name', style='green')
    parts.add_column('Type')
    parts.add_column('Core')
    parts.add_column('Offset', style='yellow', justify='right')
    parts.add_column('End', style='yellow', justify='right')
    parts.add_column('Size', justify='right')
    parts.add_column('Address', style='yellow', justify='right')
    for entry in usage_data['partitions']:
        ptype = entry.get('type') or '-'
        if entry.get('subtype'):
            ptype = f'{ptype}/{entry.get("subtype")}'
        parts.add_row(
            entry['region'],
            entry['kind'],
            entry['name'],
            ptype,
            str(entry.get('core') or '-'),
            entry.get('offset_hex') or '-',
            entry.get('end_offset_hex') or '-',
            _format_size(entry.get('size_bytes')),
            entry.get('address_hex') or '-',
        )
    console.print(parts)

    if usage_data['gaps']:
        gaps = Table(title='Free Gaps', show_header=True, header_style='bold magenta')
        gaps.add_column('Region', style='cyan')
        gaps.add_column('Offset', style='yellow', justify='right')
        gaps.add_column('End', style='yellow', justify='right')
        gaps.add_column('Size', justify='right')
        for gap in usage_data['gaps']:
            gaps.add_row(
                gap['region'],
                gap.get('offset_hex') or '-',
                gap.get('end_offset_hex') or '-',
                _format_size(gap.get('size_bytes')),
            )
        console.print(gaps)

    if usage_data['overlaps']:
        overlaps = Table(title='Overlaps', show_header=True, header_style='bold red')
        overlaps.add_column('Region', style='cyan')
        overlaps.add_column('Entries', style='green')
        overlaps.add_column('Offset', style='yellow', justify='right')
        overlaps.add_column('End', style='yellow', justify='right')
        overlaps.add_column('Size', justify='right')
        for overlap in usage_data['overlaps']:
            overlaps.add_row(
                overlap['region'],
                ', '.join(overlap.get('entries') or []),
                overlap.get('offset_hex') or '-',
                overlap.get('end_offset_hex') or '-',
                _format_size(overlap.get('size_bytes')),
            )
        console.print(overlaps)


def print_ptab_usage_json(usage_data: Dict[str, Any], pretty: bool = True) -> None:
    if pretty:
        print(json.dumps(usage_data, indent=2))
    else:
        print(json.dumps(usage_data, separators=(',', ':')))


def ptab_usage_callback(
    sdk_ctx: SdkContext,
    board: Optional[str] = None,
    board_search_path: Optional[str] = None,
    output_format: str = 'layout',
    strict: bool = False,
) -> None:
    context = _resolve_ptab_context(
        sdk_ctx,
        board,
        board_search_path,
        strict,
        'ptab usage',
    )
    usage_data = build_ptab_usage_data(
        context['ptab_module'],
        context['prepared'],
        context['ptab_obj'],
        context['chip_config'],
        context['project_dir'],
        context['board'],
        context['chip'],
    )
    _emit_ptab_report(context['ptab_module'], context['prepared'])

    if output_format == 'json':
        print_ptab_usage_json(usage_data)
    elif output_format == 'table':
        print_ptab_usage_tables(usage_data)
    else:
        print_ptab_usage_layout(usage_data)


def _board_options() -> List[Dict[str, Any]]:
    return [
        {
            'names': ['--board'],
            'help': 'Board name. Falls back to .project.toml when omitted.',
            'default': None,
        },
        {
            'names': ['--board-search-path', '--board_search_path'],
            'help': 'Additional board search path.',
            'default': None,
        },
    ]


def _strict_option() -> Dict[str, Any]:
    return {
        'names': ['--strict', '-s'],
        'help': 'Treat validation warnings as errors.',
        'is_flag': True,
        'default': False,
    }


def _export_options() -> List[Dict[str, Any]]:
    return _board_options() + [
        {
            'names': ['--output', '-o', 'output_path'],
            'help': 'Write the effective PTAB YAML to a file. Defaults to stdout.',
            'type': click.Path(dir_okay=False, resolve_path=False),
            'default': None,
        },
        _strict_option(),
    ]


def _usage_options() -> List[Dict[str, Any]]:
    return _board_options() + [
        {
            'names': ['--format', '-f', 'output_format'],
            'help': 'Output format: layout (default), table, or json.',
            'type': click.Choice(['layout', 'table', 'json']),
            'default': 'layout',
            'show_default': True,
        },
        _strict_option(),
    ]


def register(registry: CommandRegistry) -> None:
    registry.group(
        path='ptab',
        help='PTAB v3 export and usage inspection commands.',
    )

    registry.command(
        path='ptab/export',
        callback=ptab_export_callback,
        help='Export the effective project ptab v3 YAML after applying overlay files.',
        options=_export_options(),
    )

    registry.command(
        path='ptab/usage',
        callback=ptab_usage_callback,
        help='Show effective PTAB v3 storage and execution memory usage.',
        options=_usage_options(),
    )

    registry.command(
        path='ptab-export',
        callback=ptab_export_callback,
        help='Export the effective project ptab v3 YAML after applying overlay files.',
        options=_export_options(),
        hidden=True,
    )
