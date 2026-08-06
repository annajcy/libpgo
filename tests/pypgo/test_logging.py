"""Tests for the pypgo C++ backend logging controls."""

import pytest

import pypgo


def test_set_and_get_log_level_round_trip():
    previous = pypgo.get_log_level()
    for level in ("off", "warn", "info"):
        pypgo.set_log_level(level)
        assert pypgo.get_log_level() == level
    pypgo.set_log_level(previous)


def test_invalid_log_level_raises():
    with pytest.raises(ValueError, match="invalid log level"):
        pypgo.set_log_level("verbose")


def test_quiet_cpp_logs_restores_previous_level():
    previous = pypgo.get_log_level()
    pypgo.set_log_level("info")
    with pypgo.quiet_cpp_logs():
        assert pypgo.get_log_level() == "off"
    assert pypgo.get_log_level() == "info"
    pypgo.set_log_level(previous)
