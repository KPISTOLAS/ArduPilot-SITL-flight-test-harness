#!/usr/bin/env python3
"""Run a YAML scenario against SITL and save telemetry/plot.

Usage:
    python scripts/run_scenario.py scenarios/basic_mission.yaml
    python scripts/run_scenario.py scenarios/gps_loss.yaml --show
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.connection import close_vehicle, connect_vehicle
from harness.mission import load_scenario, run_scenario
from harness.sitl import SitlConfig, SitlInstance
from harness.viewer import plot_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Run one flight scenario on SITL")
    parser.add_argument("scenario", type=Path, help="Path to scenario YAML")
    parser.add_argument("--show", action="store_true", help="Show plot window")
    parser.add_argument("--speedup", type=float, default=1.0)
    parser.add_argument("--backend", default="auto", choices=["auto", "dronekit", "sim_vehicle"])
    parser.add_argument("--reports", type=Path, default=ROOT / "reports")
    args = parser.parse_args()

    scenario = load_scenario(args.scenario)
    cfg = SitlConfig(
        vehicle=scenario.get("vehicle", "copter"),
        speedup=float(scenario.get("speedup", args.speedup)),
        backend=args.backend,
    )

    with SitlInstance(cfg) as sitl:
        vehicle = connect_vehicle(sitl.connection_string)
        try:
            recorder = run_scenario(
                vehicle,
                scenario,
                record=True,
                reports_dir=args.reports,
            )
            plot_path(
                recorder.samples,
                out_png=args.reports / f"{scenario.get('name', 'flight')}_path.png",
                title=scenario.get("name", "flight"),
                show=args.show,
            )
            print(f"[run] samples={len(recorder.samples)} reports={args.reports}")
            return 0
        finally:
            close_vehicle(vehicle)


if __name__ == "__main__":
    raise SystemExit(main())
