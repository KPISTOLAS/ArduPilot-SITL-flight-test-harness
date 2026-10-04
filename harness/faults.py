"""Inject simulation faults via ArduPilot parameters."""

from __future__ import annotations

import time
from typing import Any, Mapping


def set_param(vehicle: Any, name: str, value: float, retries: int = 3) -> bool:
    """
    Set one parameter (best-effort).

    Avoids long DroneKit parameter-read deadlocks by using a short write loop
    and treating confirmation as optional.
    """
    for attempt in range(1, retries + 1):
        try:
            vehicle.parameters[name] = float(value)
        except Exception as exc:
            print(f"[param] set {name}={value} write failed ({attempt}/{retries}): {exc}")
            time.sleep(0.5)
            continue

        time.sleep(0.3)
        try:
            # Bound how long we block on a readback.
            current = float(vehicle.parameters[name])
            if abs(current - float(value)) < 1e-3:
                print(f"[param] {name}={current}")
                return True
            print(f"[param] {name} readback={current}, wanted={value}")
        except Exception as exc:
            # Write may still have worked; don't hang forever.
            print(f"[param] {name} write sent (no readback): {exc}")
            return True
        time.sleep(0.3)
    print(f"[param] WARNING: could not confirm {name}={value}")
    return False


def set_params(vehicle: Any, params: Mapping[str, float]) -> None:
    """Set many parameters."""
    for name, value in params.items():
        set_param(vehicle, name, value)


def inject_gps_loss(vehicle: Any, enable: bool = True) -> None:
    """Disable / enable GPS in SITL. Param: SIM_GPS_DISABLE (1=loss)."""
    set_param(vehicle, "SIM_GPS_DISABLE", 1.0 if enable else 0.0)


def inject_rc_loss(vehicle: Any, enable: bool = True) -> None:
    """Simulate RC fail. Param: SIM_RC_FAIL (1=fail)."""
    set_param(vehicle, "SIM_RC_FAIL", 1.0 if enable else 0.0)


def inject_battery_low(vehicle: Any, voltage: float = 9.5) -> None:
    """Force a low battery reading in SITL when supported."""
    set_param(vehicle, "SIM_BATT_VOLTAGE", float(voltage))


def clear_common_faults(vehicle: Any) -> None:
    """Reset the most common injected faults (best-effort, non-fatal)."""
    try:
        inject_gps_loss(vehicle, False)
    except Exception:
        pass
    try:
        inject_rc_loss(vehicle, False)
    except Exception:
        pass
