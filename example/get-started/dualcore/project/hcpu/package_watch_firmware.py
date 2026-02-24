#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Watch Firmware Packaging Script
This script copies watch firmware bin files to watchOS/sys directory
"""

import os
import sys
import shutil

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Define paths relative to script directory
BUILD_PATH = os.path.join(SCRIPT_DIR, "build_sf32lb56w-watch_hcpu")
SYS_DIR = os.path.join(SCRIPT_DIR, "watchOS", "sys")
WATCHFACE_DIR = os.path.join(SCRIPT_DIR, "watchOS", "watchface")
JSROOT_DIR = os.path.join(SCRIPT_DIR, "..", "jsroot")


def print_color(message, color="white"):
    """Print colored message to console"""
    colors = {
        "green": "\033[92m",
        "yellow": "\033[93m",
        "cyan": "\033[96m",
        "red": "\033[91m",
        "reset": "\033[0m"
    }
    # Check if terminal supports colors
    if sys.platform == "win32":
        # Enable ANSI colors on Windows
        os.system("")
    print(f"{colors.get(color, '')}{message}{colors['reset']}")


def package_firmware():
    """Copy watch firmware bin files to watchOS/sys directory"""

    print_color("Starting watch firmware packaging...", "green")

    # Define source paths
    main_bin = os.path.join(BUILD_PATH, "main.bin")
    ftab_bin = os.path.join(BUILD_PATH, "ftab", "ftab.bin")
    lcpu_bin = os.path.join(BUILD_PATH, "lcpu", "lcpu.bin", "ER_IROM2.bin")
    lcpu_patch_bin = os.path.join(BUILD_PATH, "lcpu_patch", "lcpu_rom_patch_copy_hex2bin.bin")
    bootloader_bin = os.path.join(BUILD_PATH, "bootloader", "bootloader.bin")

    print_color(f"Build path: {BUILD_PATH}", "cyan")
    print_color(f"Output path: {SYS_DIR}", "cyan")

    # Files to copy: (source_path, destination_name, description)
    files_to_copy = [
        (main_bin, "hcpu.bin", "main.bin -> hcpu.bin"),
        (ftab_bin, "ftab.bin", "ftab.bin copied"),
        (lcpu_bin, "lcpu.bin", "ER_IROM2.bin -> lcpu.bin"),
        (lcpu_patch_bin, "lcpu_patch.bin", "lcpu_rom_patch_copy_hex2bin.bin -> lcpu_patch.bin"),
        (bootloader_bin, "bootloader.bin", "bootloader.bin copied"),
    ]

    # Create sys directory if not exists
    print_color("Checking output directory...", "yellow")
    os.makedirs(SYS_DIR, exist_ok=True)

    # Copy files
    for src_path, dest_name, description in files_to_copy:
        print_color(f"Copying {dest_name}...", "yellow")
        if os.path.exists(src_path):
            shutil.copy2(src_path, os.path.join(SYS_DIR, dest_name))
            print_color(f"  ✓ {description}", "green")
        else:
            print_color(f"Error: {dest_name} not found at: {src_path}", "red")
            return False

    # Verify contents
    print_color("Verifying copied files...", "yellow")
    for _, dest_name, _ in files_to_copy:
        file_path = os.path.join(SYS_DIR, dest_name)
        if os.path.isfile(file_path):
            file_size = os.path.getsize(file_path)
            print_color(f"  - {dest_name} ({file_size} bytes)", "cyan")

    print_color("Firmware bin files copied successfully!", "green")
    print_color(f"Output directory: {SYS_DIR}", "cyan")

    return True


def copy_watchface_files():
    """Copy jsroot files to watchOS/watchface directory"""

    print_color("Starting watchface files copy...", "green")
    print_color(f"Source path: {JSROOT_DIR}", "cyan")
    print_color(f"Output path: {WATCHFACE_DIR}", "cyan")

    # Check if jsroot directory exists
    if not os.path.exists(JSROOT_DIR):
        print_color(f"Warning: jsroot directory not found at: {JSROOT_DIR}, skipping watchface copy.", "yellow")
        return True

    # Create watchface directory if not exists
    print_color("Checking output directory...", "yellow")
    os.makedirs(WATCHFACE_DIR, exist_ok=True)

    # Copy all files and subdirectories from jsroot to watchface
    print_color("Copying files...", "yellow")
    copied_count = 0

    for item in os.listdir(JSROOT_DIR):
        src_path = os.path.join(JSROOT_DIR, item)
        dest_path = os.path.join(WATCHFACE_DIR, item)

        if os.path.isfile(src_path):
            shutil.copy2(src_path, dest_path)
            print_color(f"  ✓ {item}", "green")
            copied_count += 1
        elif os.path.isdir(src_path):
            # Remove existing directory if exists
            if os.path.exists(dest_path):
                shutil.rmtree(dest_path)
            shutil.copytree(src_path, dest_path)
            print_color(f"  ✓ {item}/ (directory)", "green")
            copied_count += 1

    print_color(f"Watchface files copy completed! ({copied_count} items)", "green")
    print_color(f"Output directory: {WATCHFACE_DIR}", "cyan")

    return True


def main():
    success = True

    # Step 1: Copy firmware bin files
    if not package_firmware():
        success = False

    print()

    # Step 2: Copy watchface files
    if not copy_watchface_files():
        success = False

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
