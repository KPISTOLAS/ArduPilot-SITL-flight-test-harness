"""Simple parameter-sweep helper.

Example:
    from harness.sweep import expand_grid, summarize_results
    combos = expand_grid({"WPNAV_SPEED": [500, 800], "WPNAV_ACCEL": [100, 200]})
"""

from __future__ import annotations

import itertools
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence


def expand_grid(matrix: Mapping[str, Sequence[Any]]) -> List[Dict[str, Any]]:
    """Cartesian product of parameter values."""
    if not matrix:
        return [{}]
    keys = list(matrix.keys())
    values = [list(matrix[k]) for k in keys]
    return [dict(zip(keys, combo)) for combo in itertools.product(*values)]


def summarize_results(results: Iterable[Mapping[str, Any]], out_json: str | Path | None = None) -> List[dict]:
    """
    Normalize sweep results into a list of dicts and optionally write JSON.

    Each result should contain at least: params (dict), passed (bool), error (str|None)
    """
    rows = []
    for r in results:
        rows.append(
            {
                "params": dict(r.get("params") or {}),
                "passed": bool(r.get("passed")),
                "error": r.get("error"),
                "name": r.get("name"),
            }
        )
    if out_json is not None:
        path = Path(out_json)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8") as f:
            json.dump(rows, f, indent=2)
    return rows


def format_table(rows: Sequence[Mapping[str, Any]]) -> str:
    """Pretty ASCII table for terminal output."""
    if not rows:
        return "(no results)"
    lines = ["passed | params", "-" * 60]
    for r in rows:
        params = ", ".join(f"{k}={v}" for k, v in sorted((r.get("params") or {}).items()))
        flag = "PASS" if r.get("passed") else "FAIL"
        err = r.get("error")
        extra = f" ({err})" if err else ""
        lines.append(f"{flag:6} | {params}{extra}")
    return "\n".join(lines)
