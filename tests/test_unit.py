"""Unit tests that do not need SITL."""

from __future__ import annotations

from pathlib import Path

from harness.geo import haversine_m, offset_ned_to_latlon
from harness.mission import load_scenario
from harness.viewer import plot_path


ROOT = Path(__file__).resolve().parents[1]


def test_scenario_file_exists():
    path = ROOT / "scenarios" / "basic_mission.yaml"
    assert path.is_file()


def test_load_basic_scenario():
    data = load_scenario(ROOT / "scenarios" / "basic_mission.yaml")
    assert data["name"] == "basic_square"
    assert len(data["waypoints_ned_m"]) == 4
    assert data["end"] == "RTL"


def test_offset_and_distance_roundtrip():
    home_lat, home_lon = -35.363261, 149.165230
    lat, lon = offset_ned_to_latlon(home_lat, home_lon, north_m=40, east_m=0)
    dist = haversine_m(home_lat, home_lon, lat, lon)
    assert 38.0 < dist < 42.0


def test_plot_path_saves_png(tmp_path):
    samples = [
        {"t": 1.0, "lat": -35.36, "lon": 149.16, "alt": 0.0},
        {"t": 2.0, "lat": -35.361, "lon": 149.161, "alt": 5.0},
        {"t": 3.0, "lat": -35.362, "lon": 149.162, "alt": 10.0},
    ]
    out = tmp_path / "path.png"
    saved = plot_path(samples, out_png=out, show=False)
    assert saved is not None and saved.is_file()
