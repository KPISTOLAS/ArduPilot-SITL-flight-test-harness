"""Simple geo helpers (NED offsets ↔ lat/lon)."""

from __future__ import annotations

import math
from typing import Tuple

# Approximate meters per degree near the equator; scaled by latitude for lon.
_METERS_PER_DEG_LAT = 111319.5


def offset_ned_to_latlon(
    home_lat: float,
    home_lon: float,
    north_m: float,
    east_m: float,
) -> Tuple[float, float]:
    """Convert north/east meters relative to home into absolute lat/lon."""
    dlat = north_m / _METERS_PER_DEG_LAT
    cos_lat = math.cos(math.radians(home_lat))
    # Avoid divide-by-zero at the poles (irrelevant for SITL home).
    cos_lat = cos_lat if abs(cos_lat) > 1e-6 else 1e-6
    dlon = east_m / (_METERS_PER_DEG_LAT * cos_lat)
    return home_lat + dlat, home_lon + dlon


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in meters between two WGS84 points."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))
