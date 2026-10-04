"""Pytest fixtures shared by all tests."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from harness.connection import close_vehicle, connect_vehicle
from harness.sitl import SitlConfig, SitlInstance

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = ROOT / "scenarios"
REPORTS = ROOT / "reports"


def pytest_configure(config):
    config.addinivalue_line("markers", "sitl: tests that start ArduPilot SITL")


@pytest.fixture(scope="session")
def project_root() -> Path:
    return ROOT


@pytest.fixture(scope="session")
def scenarios_dir() -> Path:
    return SCENARIOS


@pytest.fixture
def reports_dir(tmp_path) -> Path:
    # Prefer repo reports/ when not in CI isolation; tmp for unit cleanliness.
    out = REPORTS
    out.mkdir(parents=True, exist_ok=True)
    return out


@pytest.fixture
def sitl():
    """Start SITL for one test, always stop afterward."""
    if os.environ.get("SKIP_SITL", "").lower() in ("1", "true", "yes"):
        pytest.skip("SKIP_SITL set")

    speedup = float(os.environ.get("SIM_SPEEDUP", "1"))
    backend = os.environ.get("SITL_BACKEND", "auto")
    instance = SitlInstance(
        SitlConfig(
            vehicle=os.environ.get("SITL_VEHICLE", "copter"),
            speedup=speedup,
            backend=backend,
        )
    )
    try:
        instance.start()
    except Exception as exc:  # noqa: BLE001
        pytest.skip(f"Could not start SITL: {exc}")

    try:
        yield instance
    finally:
        instance.stop()


@pytest.fixture
def vehicle(sitl):
    """Connected DroneKit vehicle for one test."""
    v = connect_vehicle(sitl.connection_string, wait_ready=True)
    try:
        yield v
    finally:
        close_vehicle(v)
