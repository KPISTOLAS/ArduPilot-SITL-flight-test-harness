"""Unit tests for parameter sweep helpers."""

from __future__ import annotations

from harness.sweep import expand_grid, format_table, summarize_results


def test_expand_grid():
    combos = expand_grid({"A": [1, 2], "B": [10]})
    assert combos == [{"A": 1, "B": 10}, {"A": 2, "B": 10}]


def test_summarize_and_table(tmp_path):
    rows = summarize_results(
        [
            {"params": {"WPNAV_SPEED": 500}, "passed": True},
            {"params": {"WPNAV_SPEED": 9999}, "passed": False, "error": "timeout"},
        ],
        out_json=tmp_path / "sweep.json",
    )
    assert rows[0]["passed"] is True
    text = format_table(rows)
    assert "PASS" in text and "FAIL" in text
