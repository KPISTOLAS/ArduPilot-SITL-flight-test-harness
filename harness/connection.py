"""Connect to a vehicle via DroneKit."""

from __future__ import annotations

import time
from typing import Any

import harness.compat  # noqa: F401  - patch collections before dronekit


def connect_vehicle(
    connection_string: str,
    wait_ready: bool = True,
    baud: int = 115200,
    heartbeat_timeout_s: float = 30.0,
    retries: int = 3,
) -> Any:
    """
    Connect to SITL / autopilot and return a DroneKit Vehicle.

    Retries a few times because SITL may still be booting when we connect.
    """
    from dronekit import connect

    last_err: Exception | None = None
    for attempt in range(1, retries + 1):
        vehicle = None
        try:
            print(
                f"[connect] attempt {attempt}/{retries} -> {connection_string} "
                f"(heartbeat_timeout={heartbeat_timeout_s}s)"
            )
            vehicle = connect(
                connection_string,
                wait_ready=False,
                baud=baud,
                heartbeat_timeout=heartbeat_timeout_s,
            )

            # Wait for first heartbeat.
            deadline = time.time() + heartbeat_timeout_s
            while time.time() < deadline:
                hb = getattr(vehicle, "last_heartbeat", None)
                if hb is not None and hb < 5:
                    break
                time.sleep(0.2)
            else:
                raise TimeoutError("Connected socket but no heartbeat received")

            if wait_ready:
                try:
                    vehicle.wait_ready(
                        "gps_0",
                        "armed",
                        "mode",
                        "attitude",
                        timeout=30,
                    )
                except Exception as exc:
                    print(f"[connect] wait_ready partial: {exc}")

            # Location can lag a second after GPS appears.
            deadline = time.time() + 20
            while time.time() < deadline:
                loc = vehicle.location.global_relative_frame
                if loc is not None and loc.lat is not None and loc.alt is not None:
                    break
                time.sleep(0.3)

            print(
                f"[connect] OK mode={vehicle.mode.name} armed={vehicle.armed} "
                f"ekf_ok={getattr(vehicle, 'ekf_ok', None)}"
            )
            return vehicle
        except Exception as exc:  # noqa: BLE001
            last_err = exc
            print(f"[connect] failed: {exc}")
            if vehicle is not None:
                try:
                    vehicle.close()
                except Exception:
                    pass
            time.sleep(2.0 * attempt)

    raise RuntimeError(
        f"Failed to connect to {connection_string} after {retries} attempts: {last_err}"
    )


def close_vehicle(vehicle: Any) -> None:
    """Close a DroneKit connection safely."""
    if vehicle is None:
        return
    try:
        vehicle.close()
    except Exception:
        pass
