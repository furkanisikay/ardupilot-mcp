"""Tests for the real-crash benchmark: the manifest and the scorer."""

from __future__ import annotations

import json
import os

import pytest

from ardupilot_mcp.checks.util import make_finding
from ardupilot_mcp.model import DiagnosisReport, Severity
from bench.scoring import CATEGORY_CHECKS, finding_categories, score_report, summarize
from tests.helpers import make_flight_log

_MANIFEST = os.path.join(os.path.dirname(os.path.dirname(__file__)), "bench", "real_crashes.json")


def _report(*findings) -> DiagnosisReport:
    from ardupilot_mcp.orchestrator import diagnose

    base = diagnose(make_flight_log({}))
    return base.model_copy(update={"findings": list(findings)})


def _f(check: str, sev: Severity, title: str = "t"):
    return make_finding(check, sev, title, "x")


def test_manifest_entries_are_complete_and_unique():
    with open(_MANIFEST, encoding="utf-8") as fh:
        entries = json.load(fh)["entries"]
    assert len(entries) >= 20
    ids = [e["id"] for e in entries]
    assert len(ids) == len(set(ids))
    for e in entries:
        assert len(e["sha256"]) == 64 and e["source_url"].startswith("https://")
        assert e["label_post"].startswith("https://discuss.ardupilot.org/t/")
        assert e["category"] in CATEGORY_CHECKS
        assert all(s in CATEGORY_CHECKS for s in e["secondary"])
        assert e["confidence"] in ("confirmed", "expert")


def test_top_finding_from_the_right_check_is_a_hit():
    entry = {"id": "x", "category": "vibration", "secondary": []}
    s = score_report(entry, _report(_f("vibration", Severity.WARN), _f("gps", Severity.CRITICAL)))
    assert s.hit and s.top3
    assert s.false_criticals == ["gps"]


def test_info_findings_do_not_count_as_the_top():
    entry = {"id": "x", "category": "gps", "secondary": []}
    s = score_report(entry, _report(_f("events", Severity.INFO), _f("gps", Severity.WARN)))
    assert s.hit


def test_events_finding_is_judged_by_its_subsystem():
    assert finding_categories("events", "Error: EKFCHECK (ECode 2)") == {"ekf"}
    assert finding_categories("events", "Error: FAILSAFE_RADIO (ECode 1)") == {"rc_failsafe_link"}
    entry = {"id": "x", "category": "ekf", "secondary": []}
    s = score_report(entry, _report(_f("events", Severity.CRITICAL, "Error: EKFCHECK (ECode 2)")))
    assert s.hit and s.false_criticals == []


def test_secondary_cause_is_not_a_false_critical():
    entry = {"id": "x", "category": "configuration", "secondary": ["ekf"]}
    s = score_report(entry, _report(_f("ekf", Severity.CRITICAL)))
    assert not s.hit and s.false_criticals == []


def test_not_a_fault_counts_every_critical_and_is_not_scored():
    entry = {"id": "x", "category": "not_a_fault", "secondary": []}
    s = score_report(entry, _report(_f("compass", Severity.CRITICAL), _f("gps", Severity.WARN)))
    assert s.hit is None and s.top3 is None
    assert s.false_criticals == ["compass"]


def test_summary_counts():
    entry_hit = {"id": "a", "category": "gps", "secondary": []}
    entry_miss = {"id": "b", "category": "gps", "secondary": []}
    scores = [
        score_report(entry_hit, _report(_f("gps", Severity.WARN))),
        score_report(entry_miss, _report(_f("vibration", Severity.CRITICAL))),
    ]
    out = summarize(scores)
    assert out["scored"] == 2 and out["top1_hits"] == 1
    assert out["false_criticals_by_check"] == {"vibration": 1}


def test_crash_impact_markers_are_not_false_criticals():
    entry = {"id": "x", "category": "thrust_loss_motor_esc", "secondary": []}
    s = score_report(
        entry,
        _report(
            _f("events", Severity.CRITICAL, "Error: CRASH_CHECK (ECode 1)"),
            _f("firmware", Severity.CRITICAL, "Firmware detected a crash and disarmed"),
            _f("firmware", Severity.CRITICAL, "Mode changed by crash failsafe"),
        ),
    )
    assert s.false_criticals == []


def test_runner_skips_an_unreadable_log(tmp_path, monkeypatch, capsys):
    import bench.run as run

    with open(_MANIFEST, encoding="utf-8") as fh:
        first = json.load(fh)["entries"][0]
    (tmp_path / f"{first['id']}.bin").write_bytes(b"not a log")
    monkeypatch.setattr(run, "_sha256", lambda _p: first["sha256"])

    def _boom(path):
        raise ValueError(f"could not open DataFlash log {path!r}")

    monkeypatch.setattr(run, "parse_bin", _boom)
    assert run.main(["--logs", str(tmp_path)]) == 0
    assert f"skipped {first['id']}: could not analyse" in capsys.readouterr().err


def test_runner_refuses_to_write_results_from_a_partial_run(tmp_path, capsys):
    import bench.run as run

    out = tmp_path / "results.json"
    assert run.main(["--logs", str(tmp_path), "--json", str(out)]) == 1
    assert not out.exists()
    assert "pass --allow-partial" in capsys.readouterr().err

    assert run.main(["--logs", str(tmp_path), "--json", str(out), "--allow-partial"]) == 0
    summary = json.loads(out.read_text(encoding="utf-8"))["summary"]
    assert summary["logs"] == 0
    assert set(summary["measured_on"]) == {"version", "revision"}


def test_bench_table_rejects_results_that_miss_logs(tmp_path, monkeypatch):
    import scripts.gen_docs as gen

    results = json.loads(gen.BENCH_RESULTS.read_text(encoding="utf-8"))
    results["summary"]["logs"] -= 1
    partial = tmp_path / "results.json"
    partial.write_text(json.dumps(results), encoding="utf-8")
    monkeypatch.setattr(gen, "BENCH_RESULTS", partial)
    with pytest.raises(ValueError, match="re-run python -m bench.run"):
        gen.render_bench()


def test_title_routed_categories_cover_dead_reckoning_internal_error_and_brownout():
    assert finding_categories("firmware", "Firmware fell back to dead reckoning") == {"ekf", "gps"}
    assert finding_categories("firmware", "Mode changed by dead-reckoning failsafe") == {"ekf", "gps"}
    assert finding_categories("events", "Error: FAILSAFE_DEADRECKON (ECode 1)") == {"ekf", "gps"}
    assert finding_categories("events", "Error: INTERNAL_ERROR (ECode 1)") == {"firmware_bug"}
    assert finding_categories("integrity", "Recording ended while armed") == {"power_brownout"}
    assert finding_categories("integrity", "Log truncated") == set()
