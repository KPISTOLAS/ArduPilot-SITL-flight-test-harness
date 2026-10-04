#!/usr/bin/env python3
"""Run the same scenario across a parameter grid and print a summary.

Example:
    python scripts/run_param_sweep.py scenarios/basic_mission.yaml \\
        --param WPNAV_SPEED=500,800 --param WPNAV_ACCEL=100
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
from harness.sweep import expand_grid, format_table, summarize_results


def _parse_param(text: str) -> tuple[str, list]:
    # WPNAV_SPEED=500,800
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"Expected NAME=v1,v2 got {text}")
    name, raw = text.split("=", 1)
    values = []
    for part in raw.split(","):
        part = part.strip()
        try:
            values.append(int(part) if part.isdigit() else float(part))
        except ValueError:
            values.append(part)
    return name.strip(), values


def main() -> int:
    parser = argparse.ArgumentParser(description="Parameter sweep over one scenario")
    parser.add_argument("scenario", type=Path)
    parser.add_argument(
        "--param",
        action="append",
        default=[],
        type=_parse_param,
        help="NAME=v1,v2 (repeatable)",
    )
    parser.add_argument("--reports", type=Path, default=ROOT / "reports" / "sweep")
    args = parser.parse_args()

    base = load_scenario(args.scenario)
    matrix = {name: values for name, values in args.param}
    combos = expand_grid(matrix)
    if not combos:
        combos = [{}]

    results = []
    args.reports.mkdir(parents=True, exist_ok=True)

    for i, combo in enumerate(combos, start=1):
        scenario = dict(base)
        scenario["name"] = f"{base.get('name', 'flight')}_sweep{i}"
        params = dict(scenario.get("params") or {})
        params.update(combo)
        scenario["params"] = params

        print(f"\n=== Sweep {i}/{len(combos)} params={combo} ===")
        cfg = SitlConfig(vehicle=scenario.get("vehicle", "copter"), speedup=1.0)
        passed = False
        err = None
        try:
            with SitlInstance(cfg) as sitl:
                vehicle = connect_vehicle(sitl.connection_string)
                try:
                    run_scenario(vehicle, scenario, record=True, reports_dir=args.reports)
                    passed = True
                finally:
                    close_vehicle(vehicle)
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
            print(f"[sweep] FAIL: {exc}")

        results.append({"params": combo, "passed": passed, "error": err, "name": scenario["name"]})

    rows = summarize_results(results, out_json=args.reports / "summary.json")
    print("\n" + format_table(rows))
    return 0 if all(r["passed"] for r in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
