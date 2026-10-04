#!/usr/bin/env python3
"""Manual check: start SITL, connect, print state, stop.

Usage (from project root, venv active):
    python scripts/run_sitl_smoke.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.connection import close_vehicle, connect_vehicle
from harness.sitl import SitlConfig, SitlInstance


def main() -> int:
    cfg = SitlConfig(vehicle="copter", speedup=1.0, backend="auto")
    print(f"[smoke] Starting SITL (backend={cfg.backend})...")
    with SitlInstance(cfg) as sitl:
        print(f"[smoke] connection_string = {sitl.connection_string}")
        vehicle = connect_vehicle(sitl.connection_string)
        try:
            for i in range(8):
                loc = vehicle.location.global_relative_frame
                gps = vehicle.gps_0
                sats = gps.satellites_visible if gps else None
                fix = gps.fix_type if gps else None
                alt = loc.alt if loc and loc.alt is not None else 0.0
                print(
                    f"[smoke] mode={vehicle.mode.name} armed={vehicle.armed} "
                    f"alt={alt:.1f} gps_fix={fix} sats={sats} ekf_ok={vehicle.ekf_ok}"
                )
                time.sleep(1.0)
            print("[smoke] OK")
            return 0
        finally:
            close_vehicle(vehicle)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"[smoke] FAILED: {exc}", file=sys.stderr)
        raise SystemExit(1)
