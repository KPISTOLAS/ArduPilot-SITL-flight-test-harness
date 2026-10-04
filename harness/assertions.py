"""Flight assertions used by tests and scenario runner."""

from __future__ import annotations

import time
from typing import Any, Iterable, Sequence

from harness.geo import haversine_m


def wait_for(
    predicate,
    timeout_s: float,
    poll_s: float = 0.25,
    description: str = "condition",
) -> None:
    """Poll until predicate() is true or raise AssertionError."""
    deadline = time.time() + timeout_s
    last_msg = ""
    while time.time() < deadline:
        ok, last_msg = predicate()
        if ok:
            return
        time.sleep(poll_s)
    raise AssertionError(f"Timed out after {timeout_s}s waiting for {description}: {last_msg}")


def assert_mode(vehicle: Any, expected_mode: str, timeout_s: float = 15.0) -> None:
    expected = expected_mode.upper()

    def _pred():
        current = str(vehicle.mode.name).upper() if vehicle.mode else "?"
        return current == expected, f"mode={current}"

    wait_for(_pred, timeout_s=timeout_s, description=f"mode {expected}")


def assert_armed(vehicle: Any, armed: bool = True, timeout_s: float = 15.0) -> None:
    def _pred():
        return bool(vehicle.armed) == armed, f"armed={vehicle.armed}"

    wait_for(_pred, timeout_s=timeout_s, description=f"armed={armed}")


def assert_altitude_in_range(
    vehicle: Any,
    min_m: float,
    max_m: float,
    timeout_s: float = 0.0,
) -> None:
    """
    If timeout_s == 0, check once. Otherwise wait until altitude enters range.
    """
    def _pred():
        alt = float(vehicle.location.global_relative_frame.alt)
        ok = min_m <= alt <= max_m
        return ok, f"alt={alt:.2f}"

    if timeout_s <= 0:
        ok, msg = _pred()
        if not ok:
            raise AssertionError(f"Altitude out of range [{min_m}, {max_m}]: {msg}")
        return
    wait_for(_pred, timeout_s=timeout_s, description=f"altitude in [{min_m}, {max_m}]")


def assert_reached_waypoint(
    vehicle: Any,
    lat: float,
    lon: float,
    radius_m: float,
    timeout_s: float = 60.0,
) -> None:
    def _pred():
        loc = vehicle.location.global_relative_frame
        dist = haversine_m(loc.lat, loc.lon, lat, lon)
        return dist <= radius_m, f"dist={dist:.1f}m"

    wait_for(
        _pred,
        timeout_s=timeout_s,
        description=f"reach ({lat:.6f},{lon:.6f}) within {radius_m}m",
    )


def assert_altitudes_within(
    altitudes: Sequence[float],
    min_m: float,
    max_m: float,
) -> None:
    if not altitudes:
        raise AssertionError("No altitude samples to assert")
    lo, hi = min(altitudes), max(altitudes)
    if lo < min_m or hi > max_m:
        raise AssertionError(
            f"Altitude envelope [{lo:.2f}, {hi:.2f}] outside allowed [{min_m}, {max_m}]"
        )


def assert_modes_seen(samples: Iterable[dict], required: Sequence[str]) -> None:
    seen = {str(s.get("mode", "")).upper() for s in samples}
    missing = [m.upper() for m in required if m.upper() not in seen]
    if missing:
        raise AssertionError(f"Expected modes {required}, missing {missing}; seen={sorted(seen)}")
