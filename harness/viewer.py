"""Flight-path visualization helpers."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Optional, Sequence


def plot_path(
    samples: Sequence[Dict[str, Any]],
    out_png: str | Path | None = None,
    title: str = "Flight path",
    show: bool = False,
) -> Optional[Path]:
    """
    Plot 2D lat/lon trail and altitude vs time.

    Saves PNG when out_png is set. Set show=True for interactive window.
    """
    if not samples:
        raise ValueError("No telemetry samples to plot")

    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    lats = [s["lat"] for s in samples if s.get("lat") is not None]
    lons = [s["lon"] for s in samples if s.get("lon") is not None]
    alts = [s["alt"] for s in samples if s.get("alt") is not None]
    ts = [s["t"] for s in samples if s.get("alt") is not None]
    t0 = ts[0] if ts else 0.0
    t_rel = [t - t0 for t in ts]

    fig, (ax_map, ax_alt) = plt.subplots(1, 2, figsize=(10, 4))
    if lons and lats:
        ax_map.plot(lons, lats, "-b", linewidth=1.5, label="path")
        ax_map.scatter(lons[0], lats[0], c="green", s=40, label="start", zorder=3)
        ax_map.scatter(lons[-1], lats[-1], c="red", s=40, label="end", zorder=3)
        ax_map.set_xlabel("Longitude")
        ax_map.set_ylabel("Latitude")
        ax_map.set_title("Horizontal path")
        ax_map.legend(loc="best")
        ax_map.grid(True, alpha=0.3)
        ax_map.set_aspect("equal", adjustable="datalim")

    if t_rel and alts:
        ax_alt.plot(t_rel, alts, "-r")
        ax_alt.set_xlabel("Time (s)")
        ax_alt.set_ylabel("Rel altitude (m)")
        ax_alt.set_title("Altitude")
        ax_alt.grid(True, alpha=0.3)

    fig.suptitle(title)
    fig.tight_layout()

    saved: Optional[Path] = None
    if out_png is not None:
        saved = Path(out_png)
        saved.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(saved, dpi=120)
    if show:
        plt.show()
    plt.close(fig)
    return saved

