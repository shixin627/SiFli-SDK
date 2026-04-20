#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script to automatically update info.json with version and file lists
"""

import json
import os
import re
import sys

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# File paths (relative to script location)
HEADER_FILE = os.path.join(SCRIPT_DIR, "..", "..", "src", "modules", "model", "watch_global_data.h")
INFO_JSON = os.path.join(SCRIPT_DIR, "info.json")
SYS_DIR = os.path.join(SCRIPT_DIR, "watchOS", "sys")
WATCHFACE_DIR = os.path.join(SCRIPT_DIR, "watchOS", "watchface")


def read_version_from_header():
    """Read version number from header file"""
    try:
        with open(HEADER_FILE, 'r', encoding='utf-8') as f:
            content = f.read()

        # Extract version components using regex
        major = re.search(r'#define\s+VERSION_MAJOR\s+(\d+)', content)
        minor = re.search(r'#define\s+VERSION_MINOR\s+(\d+)', content)
        revision = re.search(r'#define\s+VERSION_REVISION\s+(\d+)', content)

        if major and minor and revision:
            version = f"{major.group(1)}.{minor.group(1)}.{revision.group(1)}"
            return version
        else:
            print("Warning: Could not find all version components in header file")
            return None
    except Exception as e:
        print(f"Error reading header file: {e}")
        return None


def get_file_list_from_sys():
    """Get list of files from sys directory"""
    try:
        if not os.path.exists(SYS_DIR):
            print(f"Warning: sys directory not found: {SYS_DIR}")
            return []

        files = []
        for filename in os.listdir(SYS_DIR):
            filepath = os.path.join(SYS_DIR, filename)
            if os.path.isfile(filepath):
                files.append(filename)

        return sorted(files)
    except Exception as e:
        print(f"Error reading sys directory: {e}")
        return []


def get_watchface_file_list():
    """Get directory structure from watchface directory"""
    try:
        if not os.path.exists(WATCHFACE_DIR):
            print(f"Warning: watchface directory not found: {WATCHFACE_DIR}")
            return []

        file_list = []

        # Walk through all subdirectories
        for root, dirs, files in os.walk(WATCHFACE_DIR):
            # Get relative path from watchface directory
            rel_path = os.path.relpath(root, WATCHFACE_DIR)

            # Process files in current directory
            for filename in sorted(files):
                if rel_path == '.':
                    # Files in root watchface directory
                    file_list.append(f"/{filename}")
                else:
                    # Files in subdirectories
                    # Convert Windows path separator to forward slash
                    rel_path_unix = rel_path.replace('\\', '/')
                    file_list.append(f"/{rel_path_unix}/{filename}")

        return sorted(file_list)
    except Exception as e:
        print(f"Error reading watchface directory: {e}")
        return []


def update_info_json(include_watchface=False):
    """Update info.json with version and file lists"""
    try:
        # Read version
        version = read_version_from_header()
        if version is None:
            print("Failed to read version, keeping existing version")
            version = None
        else:
            print(f"Version found: {version}")

        # Get file lists
        file_list = get_file_list_from_sys()
        print(f"Found {len(file_list)} files in sys directory")

        if include_watchface:
            watchface_file_list = get_watchface_file_list()
            print(f"Found {len(watchface_file_list)} files in watchface directory")
        else:
            watchface_file_list = []
            print("Skipping watchface file list (not requested).")

        # Read existing info.json
        info_data = {}
        if os.path.exists(INFO_JSON):
            try:
                with open(INFO_JSON, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    if content:
                        info_data = json.loads(content)
            except json.JSONDecodeError:
                print("Warning: info.json is invalid, creating new one")
        else:
            print("info.json not found, creating new one")

        # Update fields
        if version is not None:
            info_data['version'] = version

        if 'description' not in info_data:
            info_data['description'] = ""

        if file_list:
            info_data['fileList'] = file_list

        if include_watchface:
            if watchface_file_list:
                info_data['watchfaceFileList'] = watchface_file_list
        else:
            # Remove watchfaceFileList when watchface is not included
            info_data.pop('watchfaceFileList', None)

        # Write updated info.json
        with open(INFO_JSON, 'w', encoding='utf-8') as f:
            json.dump(info_data, f, indent=4, ensure_ascii=False)

        print(f"\nSuccessfully updated info.json:")
        if version:
            print(f"  - Version: {version}")
        print(f"  - File list: {len(file_list)} files")
        if include_watchface:
            print(f"  - Watchface file list: {len(watchface_file_list)} files")
        else:
            print(f"  - Watchface file list: (excluded)")

    except Exception as e:
        print(f"Error updating info.json: {e}")
        return False

    return True


if __name__ == "__main__":
    print("=" * 60)
    print("Updating info.json...")
    print("=" * 60)

    include_watchface = "--with-watchface" in sys.argv
    success = update_info_json(include_watchface=include_watchface)

    print("=" * 60)
    if success:
        print("Update completed successfully!")
    else:
        print("Update failed!")
    print("=" * 60)

    sys.exit(0 if success else 1)
