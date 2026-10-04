"""Record telemetry samples during a flight."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple


@dataclass
class TelemetryRecorder:
    """Collects periodic snapshots of vehicle state."""

    samples: List[Dict[str, Any]] = field(default_factory=list)
    _vehicle: Any = field(default=None, repr=False)
    _running: bool = False
    _listener: Optional[Callable] = None
    min_interval_s: float = 0.2
    _last_sample_t: float = 0.0

    def start(self, vehicle: Any) -> None:
        """Attach listeners and start recording."""
        self._vehicle = vehicle
        self._running = True
        self._last_sample_t = 0.0
        self.samples.clear()

        # DroneKit calls listeners as: fn(vehicle, attr_name, value)
        def _on_location(_vehicle, _attr_name, _value):
            self._maybe_record()

        vehicle.add_attribute_listener("location.global_relative_frame", _on_location)
        self._listener = _on_location
        self.record_once()

    def stop(self) -> None:
        self._running = False
        if self._vehicle is not None and self._listener is not None:
            try:
                self._vehicle.remove_attribute_listener(
                    "location.global_relative_frame", self._listener
                )
            except Exception:
                pass
        self._listener = None
        self._vehicle = None

    def record_once(self) -> Optional[Dict[str, Any]]:
        if self._vehicle is None:
            return None
        v = self._vehicle
        loc = v.location.global_relative_frame
        att = v.attitude
        gps = v.gps_0
        sample = {
            "t": time.time(),
            "lat": float(loc.lat) if loc else None,
            "lon": float(loc.lon) if loc else None,
            "alt": float(loc.alt) if loc else None,
            "roll": float(att.roll) if att else None,
            "pitch": float(att.pitch) if att else None,
            "yaw": float(att.yaw) if att else None,
            "mode": str(v.mode.name) if v.mode else None,
            "armed": bool(v.armed),
            "battery_voltage": float(v.battery.voltage)
            if v.battery and v.battery.voltage is not None
            else None,
            # DroneKit GPSInfo uses fix_type + satellites_visible (not satellites_count)
            "gps_fix_type": int(gps.fix_type)
            if gps and gps.fix_type is not None
            else None,
            "gps_satellites": int(gps.satellites_visible)
            if gps and gps.satellites_visible is not None
            else None,
            "ekf_ok": bool(v.ekf_ok) if hasattr(v, "ekf_ok") else None,
        }
        self.samples.append(sample)
        self._last_sample_t = sample["t"]
        return sample

    def _maybe_record(self) -> None:
        if not self._running:
            return
        now = time.time()
        if now - self._last_sample_t < self.min_interval_s:
            return
        self.record_once()

    def save_json(self, path: str | Path) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as f:
            json.dump(self.samples, f, indent=2)
        return out

    def altitudes(self) -> List[float]:
        return [s["alt"] for s in self.samples if s.get("alt") is not None]

    def path_latlon(self) -> List[Tuple[float, float]]:
        return [
            (s["lat"], s["lon"])
            for s in self.samples
            if s.get("lat") is not None and s.get("lon") is not None
        ]
