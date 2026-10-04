"""Reliable flight-mode changes for old and new ArduPilot SITL."""

from __future__ import annotations

import time
from typing import Any

import harness.compat  # noqa: F401 - patch pymavlink set_mode_apm
from pymavlink import mavutil


def _current_mode(vehicle: Any) -> str:
    try:
        return str(vehicle.mode.name).upper()
    except Exception:
        return "?"


def _mode_id(vehicle: Any, mode_name: str) -> int | None:
    mapping = getattr(vehicle, "_mode_mapping", None) or {}
    if mode_name in mapping:
        return int(mapping[mode_name])
    for key, value in mapping.items():
        if str(key).upper() == mode_name:
            return int(value)
    return None


def set_vehicle_mode(vehicle: Any, mode_name: str, timeout_s: float = 30.0) -> None:
    """
    Switch flight mode.

    For dronekit-sitl Copter 3.3, only classic SET_MODE works (not DO_SET_MODE).
    harness.compat patches pymavlink; we also send SET_MODE explicitly here.
    """
    # Ensure patch is applied even if something imported pymavlink early.
    harness.compat.patch_pymavlink_set_mode_apm()

    target = mode_name.upper()
    deadline = time.time() + timeout_s
    mode_id = _mode_id(vehicle, target)
    if mode_id is None:
        raise ValueError(
            f"Mode {target} not in vehicle mode map: "
            f"{list((getattr(vehicle, '_mode_mapping', {}) or {}).keys())}"
        )

    print(f"[mode] requesting {target} (id={mode_id})")
    master = vehicle._master

    while time.time() < deadline:
        current = _current_mode(vehicle)
        if current == target:
            print(f"[mode] now {current}")
            return

        # Classic SET_MODE — required by Copter 3.3
        try:
            master.mav.set_mode_send(
                master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id,
            )
        except Exception as exc:
            print(f"[mode] set_mode_send failed: {exc}")

        # Also use patched pymavlink helper
        try:
            master.set_mode(target)
        except Exception:
            try:
                master.set_mode(mode_id)
            except Exception:
                pass

        time.sleep(0.5)

    raise TimeoutError(f"Could not set mode {target} (still {_current_mode(vehicle)})")
