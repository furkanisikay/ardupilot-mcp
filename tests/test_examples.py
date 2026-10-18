"""The example scripts behave as their usage lines say."""

from __future__ import annotations

import pytest

from examples import generate_demo_log


@pytest.mark.parametrize("flag", ["-h", "--help"])
def test_generate_demo_log_help_prints_usage_and_writes_nothing(flag, tmp_path, monkeypatch, capsys):
    monkeypatch.chdir(tmp_path)
    with pytest.raises(SystemExit) as exc:
        generate_demo_log.main([flag])
    assert exc.value.code == 0
    assert "usage:" in capsys.readouterr().out
    assert list(tmp_path.iterdir()) == []


def test_generate_demo_log_writes_the_given_path(tmp_path):
    out = tmp_path / "demo.bin"
    assert generate_demo_log.main([str(out)]) == 0
    assert out.stat().st_size > 0
