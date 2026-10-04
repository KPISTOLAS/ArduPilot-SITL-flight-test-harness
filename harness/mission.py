"""Upload and run missions / YAML scenarios."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List

import yaml
import harness.compat  # noqa: F401  - patch collections before dronekit
from dronekit import Command, LocationGlobalRelative
from pymavlink import mavutil

from harness.assertions import assert_mode, assert_reached_waypoint
from harness.faults import clear_common_faults, set_params
from harness.geo import offset_ned_to_latlon
from harness.modes import set_vehicle_mode
from harness.telemetry import TelemetryRecorder


@dataclass
class Waypoint:
    lat: float
    lon: float
    alt: float


def load_scenario(path: str | Path) -> Dict[str, Any]:
    """Load a YAML scenario file into a dict."""
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Scenario must be a mapping: {path}")
    return data


def _home_latlon(vehicle: Any) -> tuple[float, float]:
    loc = vehicle.location.global_frame
    return float(loc.lat), float(loc.lon)


def waypoints_from_scenario(vehicle: Any, scenario: Dict[str, Any]) -> List[Waypoint]:
    """Build absolute waypoints from NED offsets or absolute lat/lon entries."""
    home_lat, home_lon = _home_latlon(vehicle)
    takeoff_alt = float(scenario.get("takeoff_alt_m", 10))
    result: List[Waypoint] = []

    for item in scenario.get("waypoints_ned_m", []) or []:
        lat, lon = offset_ned_to_latlon(
            home_lat,
            home_lon,
            float(item["north"]),
            float(item["east"]),
        )
        alt = float(item.get("alt", takeoff_alt))
        result.append(Waypoint(lat, lon, alt))

    for item in scenario.get("waypoints_llh", []) or []:
        result.append(
            Waypoint(float(item["lat"]), float(item["lon"]), float(item["alt"]))
        )

    return result


def upload_mission(vehicle: Any, waypoints: List[Waypoint], takeoff_alt_m: float) -> None:
    """
    Clear current mission and upload:
    TAKEOFF → NAV_WAYPOINTs → (mission ends; caller chooses RTL/LAND).
    """
    cmds = vehicle.commands
    cmds.clear()
    cmds.upload()
    time.sleep(0.5)
    cmds.clear()

    # Dummy waypoint required by ArduPilot mission protocol quirks with DroneKit.
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    )
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            takeoff_alt_m,
        )
    )
    for wp in waypoints:
        cmds.add(
            Command(
                0,
                0,
                0,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                0,
                0,
                0,
                0,
                0,
                0,
                wp.lat,
                wp.lon,
                wp.alt,
            )
        )

    cmds.upload()
    cmds.wait_ready()
    time.sleep(1.0)


def arm_and_takeoff(vehicle: Any, target_alt_m: float, timeout_s: float = 90.0) -> None:
    """Arm in GUIDED and climb to target relative altitude."""
    print("[mission] Waiting for GPS 3D fix + EKF...")
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        gps = vehicle.gps_0
        fix = gps.fix_type if gps else None
        ekf = bool(getattr(vehicle, "ekf_ok", False))
        print(f"[mission] ready-check fix={fix} ekf_ok={ekf} armable={vehicle.is_armable}")
        if fix is not None and fix >= 3 and (ekf or vehicle.is_armable):
            break
        time.sleep(1.0)
    else:
        gps = vehicle.gps_0
        if not (gps and gps.fix_type and gps.fix_type >= 3):
            raise TimeoutError(
                "Vehicle not ready to arm "
                f"(fix={getattr(gps, 'fix_type', None)}, ekf_ok={getattr(vehicle, 'ekf_ok', None)})"
            )
        print("[mission] WARNING: continuing without ekf_ok")

    print("[mission] GUIDED + arm...")
    set_vehicle_mode(vehicle, "GUIDED", timeout_s=30)

    deadline = time.time() + timeout_s
    while not vehicle.armed:
        if time.time() > deadline:
            raise TimeoutError("Failed to arm in GUIDED")
        if _current_mode_name(vehicle) != "GUIDED":
            set_vehicle_mode(vehicle, "GUIDED", timeout_s=10)
        try:
            vehicle._master.arducopter_arm()
        except Exception:
            vehicle.armed = True
        time.sleep(1.0)
        print(f"[mission] arming... armed={vehicle.armed} mode={_current_mode_name(vehicle)}")

    print(f"[mission] Taking off to {target_alt_m} m...")
    time.sleep(1.0)  # brief settle after arm
    try:
        vehicle.simple_takeoff(target_alt_m)
    except Exception as exc:
        print(f"[mission] simple_takeoff error: {exc}")
    try:
        vehicle._master.mav.command_long_send(
            vehicle._master.target_system,
            vehicle._master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            float(target_alt_m),
        )
    except Exception as exc:
        print(f"[mission] NAV_TAKEOFF error: {exc}")

    deadline = time.time() + timeout_s
    alt = 0.0
    while time.time() < deadline:
        loc = vehicle.location.global_relative_frame
        alt = float(loc.alt) if loc and loc.alt is not None else 0.0
        print(f"[mission] climbing... alt={alt:.2f}m mode={_current_mode_name(vehicle)}")
        if alt >= target_alt_m * 0.90:
            print("[mission] Takeoff altitude reached")
            return
        mode = _current_mode_name(vehicle)
        if mode in ("LAND", "RTL", "BRAKE"):
            raise RuntimeError(
                f"Vehicle left GUIDED during takeoff (mode={mode}). "
                "Check FS_GCS_ENABLE / FS_THR_ENABLE params."
            )
        if mode != "GUIDED":
            set_vehicle_mode(vehicle, "GUIDED", timeout_s=10)
            vehicle.simple_takeoff(target_alt_m)
        # Do NOT spam simple_goto here — it can cancel NAV_TAKEOFF on some builds.
        if alt < 0.5:
            vehicle.simple_takeoff(target_alt_m)
        time.sleep(1.0)
    raise TimeoutError(f"Takeoff timed out; last alt={alt:.2f}m")


def _current_mode_name(vehicle: Any) -> str:
    try:
        return str(vehicle.mode.name).upper()
    except Exception:
        return "?"


def start_auto_mission(vehicle: Any) -> None:
    """Switch to AUTO so the uploaded mission runs."""
    vehicle.commands.next = 0
    set_vehicle_mode(vehicle, "AUTO", timeout_s=30)


def wait_mission_near_complete(
    vehicle: Any,
    waypoints: List[Waypoint],
    radius_m: float,
    timeout_s: float,
) -> None:
    """Wait until the vehicle reaches the last waypoint."""
    if not waypoints:
        return
    last = waypoints[-1]
    assert_reached_waypoint(vehicle, last.lat, last.lon, radius_m, timeout_s=timeout_s)


def do_end_action(vehicle: Any, end: str, timeout_s: float = 120.0) -> None:
    end_u = (end or "RTL").upper()
    if end_u == "RTL":
        set_vehicle_mode(vehicle, "RTL", timeout_s=30)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            loc = vehicle.location.global_relative_frame
            alt = float(loc.alt) if loc and loc.alt is not None else 999.0
            if alt < 2.0 or not vehicle.armed:
                return
            time.sleep(0.5)
    elif end_u == "LAND":
        set_vehicle_mode(vehicle, "LAND", timeout_s=30)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if not vehicle.armed:
                return
            time.sleep(0.5)
    elif end_u in ("LOITER", "HOLD"):
        set_vehicle_mode(vehicle, "LOITER", timeout_s=30)
    else:
        raise ValueError(f"Unknown end action: {end}")


def _apply_faults_at_wp(
    vehicle: Any,
    scenario: Dict[str, Any],
    completed_wp_index: int,
    fired: set,
) -> bool:
    """
    Apply any faults scheduled after this waypoint.

    Returns True if a fault requested stopping the mission (expect_mode set).
    """
    stop_mission = False
    for idx, fault in enumerate(scenario.get("faults") or []):
        if idx in fired:
            continue
        after_wp = fault.get("after_wp")
        if after_wp is None or int(after_wp) != completed_wp_index:
            continue
        params = fault.get("set") or {}
        print(f"[fault] Applying after WP {completed_wp_index}: {params}")
        set_params(vehicle, params)
        fired.add(idx)
        duration = fault.get("duration_s")
        if duration and fault.get("block"):
            # Beginner-friendly: only sleep/clear when scenario sets block: true.
            time.sleep(float(duration))
            if "SIM_GPS_DISABLE" in params:
                set_params(vehicle, {"SIM_GPS_DISABLE": 0})
            if "SIM_RC_FAIL" in params:
                set_params(vehicle, {"SIM_RC_FAIL": 0})
        if fault.get("expect_mode"):
            expect = scenario.get("expect") or {}
            assert_mode(
                vehicle,
                fault["expect_mode"],
                timeout_s=float(expect.get("mode_timeout_s", 20)),
            )
            stop_mission = True
    return stop_mission


def goto_waypoint(
    vehicle: Any,
    wp: Waypoint,
    radius_m: float,
    timeout_s: float,
) -> None:
    """Fly to a waypoint in GUIDED, refreshing setpoints for Copter 3.3."""
    from harness.geo import haversine_m

    target = LocationGlobalRelative(wp.lat, wp.lon, wp.alt)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        mode = _current_mode_name(vehicle)
        if mode in ("RTL", "LAND", "BRAKE", "SMART_RTL"):
            raise RuntimeError(f"Left GUIDED while navigating (mode={mode})")
        if mode != "GUIDED":
            set_vehicle_mode(vehicle, "GUIDED", timeout_s=10)
        vehicle.simple_goto(target)
        loc = vehicle.location.global_relative_frame
        if loc and loc.lat is not None and loc.lon is not None:
            dist = haversine_m(loc.lat, loc.lon, wp.lat, wp.lon)
            if dist <= radius_m:
                return
        time.sleep(1.0)
    loc = vehicle.location.global_relative_frame
    dist = (
        haversine_m(loc.lat, loc.lon, wp.lat, wp.lon)
        if loc and loc.lat is not None
        else float("inf")
    )
    raise TimeoutError(f"Did not reach waypoint within {timeout_s}s (dist={dist:.1f}m)")


def run_scenario(
    vehicle: Any,
    scenario: Dict[str, Any] | str | Path,
    record: bool = True,
    reports_dir: str | Path = "reports",
) -> TelemetryRecorder:
    """
    Full scenario runner:
    load YAML → set params → arm/takeoff → AUTO mission → faults → end action.
    """
    if not isinstance(scenario, dict):
        scenario = load_scenario(scenario)

    expect = scenario.get("expect") or {}
    wp_radius = float(expect.get("wp_radius_m", 5.0))
    takeoff_alt = float(scenario.get("takeoff_alt_m", 10))
    timeout_s = float(scenario.get("timeout_s", 180))

    # Base params from scenario
    if scenario.get("params"):
        set_params(vehicle, scenario["params"])
    clear_common_faults(vehicle)

    waypoints = waypoints_from_scenario(vehicle, scenario)
    if not waypoints:
        raise ValueError("Scenario has no waypoints")

    recorder = TelemetryRecorder()
    try:
        arm_and_takeoff(vehicle, takeoff_alt)
        if record:
            recorder.start(vehicle)

        use_auto = bool(scenario.get("use_auto", False))
        fired: set = set()

        if use_auto:
            upload_mission(vehicle, waypoints, takeoff_alt_m=takeoff_alt)
            start_auto_mission(vehicle)
            for i, wp in enumerate(waypoints, start=1):
                current = _current_mode_name(vehicle)
                if current in ("RTL", "LAND", "BRAKE", "SMART_RTL"):
                    print(f"[mission] Stopping AUTO loop; mode already {current}")
                    break
                assert_reached_waypoint(vehicle, wp.lat, wp.lon, wp_radius, timeout_s=timeout_s)
                print(f"[mission] Reached WP {i}/{len(waypoints)}")
                if _apply_faults_at_wp(vehicle, scenario, i, fired):
                    break
        else:
            for i, wp in enumerate(waypoints, start=1):
                current = _current_mode_name(vehicle)
                if current in ("RTL", "LAND", "BRAKE", "SMART_RTL"):
                    print(f"[mission] Stopping waypoint loop; mode already {current}")
                    break
                print(f"[mission] Going to WP {i}: {wp}")
                goto_waypoint(vehicle, wp, wp_radius, timeout_s=timeout_s)
                print(f"[mission] Reached WP {i}/{len(waypoints)}")
                if _apply_faults_at_wp(vehicle, scenario, i, fired):
                    break

        end = scenario.get("end", "RTL")
        current = _current_mode_name(vehicle)
        if current not in ("RTL", "LAND", "BRAKE", "SMART_RTL"):
            do_end_action(vehicle, end, timeout_s=timeout_s)

        allowed = [m.upper() for m in (expect.get("final_modes") or [])]
        if expect.get("final_mode"):
            allowed.append(str(expect["final_mode"]).upper())
        if allowed:
            deadline = time.time() + float(expect.get("mode_timeout_s", 30))
            while time.time() < deadline:
                if _current_mode_name(vehicle) in allowed:
                    break
                time.sleep(0.5)
            else:
                raise AssertionError(
                    f"Expected final mode in {allowed}, got {_current_mode_name(vehicle)}"
                )

        if expect.get("alt_min_m") is not None and expect.get("alt_max_m") is not None:
            from harness.assertions import assert_altitudes_within

            assert_altitudes_within(
                recorder.altitudes(),
                float(expect["alt_min_m"]),
                float(expect["alt_max_m"]),
            )

    finally:
        if record:
            recorder.stop()
            out = Path(reports_dir)
            out.mkdir(parents=True, exist_ok=True)
            name = scenario.get("name", "flight")
            recorder.save_json(out / f"{name}_telemetry.json")

    return recorder
