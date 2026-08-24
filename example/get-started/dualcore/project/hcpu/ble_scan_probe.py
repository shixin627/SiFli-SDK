#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""List every BLE advertisement the bench can hear.

The release's BLE step only ever says "MAC X not found", which cannot tell a
dead adapter from a watch that stopped advertising. This scans for everything
and prints it, so the two are distinguishable without standing at the bench.
Read-only: nothing is written to the watch and no MAC is assigned.

  py -3 ble_scan_probe.py [--seconds 15] [--expect AA:BB:CC:DD:EE:FF]
"""

import argparse
import asyncio
import sys


async def scan(seconds):
    from bleak import BleakScanner

    seen = {}

    def detected(device, advertisement):
        rssi = getattr(advertisement, "rssi", None)
        previous = seen.get(device.address)
        # Keep the strongest sighting: RSSI swings several dB between packets.
        if previous is None or (rssi is not None and
                                (previous[1] is None or rssi > previous[1])):
            seen[device.address] = (device.name or advertisement.local_name,
                                    rssi)

    scanner = BleakScanner(detection_callback=detected)
    await scanner.start()
    try:
        await asyncio.sleep(seconds)
    finally:
        await scanner.stop()
    return seen


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--expect", default="")
    args = parser.parse_args()

    try:
        seen = asyncio.run(scan(args.seconds))
    except Exception as e:
        print("SCAN FAILED: %s: %s" % (e.__class__.__name__, e))
        return 1

    print("heard %d device(s) in %.0fs" % (len(seen), args.seconds))
    for address, (name, rssi) in sorted(
            seen.items(), key=lambda kv: kv[1][1] or -999, reverse=True):
        print("  %s  rssi=%s  name=%s" % (address, rssi, name))

    if not seen:
        print("NOTHING HEARD — the adapter started but received no "
              "advertisement at all, so this is not about the watch.")
    if args.expect:
        want = args.expect.upper()
        hit = [a for a in seen if a.upper() == want]
        print("expected %s: %s" % (want, "FOUND" if hit else "NOT FOUND"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
