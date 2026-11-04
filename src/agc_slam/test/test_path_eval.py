"""Unit tests for path_eval's frame change and error metrics (no bag, no matplotlib)."""

import math

import numpy as np
import pytest

from agc_slam.path_eval import position_errors, world_to_odom


def test_world_to_odom_translation_and_yaw():
    # Odom frame starts at world (-2, -6) facing world +y (yaw 90 deg).
    origin = (-2.0, -6.0, math.pi / 2)
    pts = world_to_odom([(-2.0, -6.0), (-2.0, -5.0), (-3.0, -6.0)], origin)
    # The start is odom 0; one metre along world +y is odom +x (ahead);
    # one metre along world -x is odom +y (to the left).
    assert pts == pytest.approx(np.array([[0, 0], [1, 0], [0, 1]]), abs=1e-9)


def test_perfect_slam_has_zero_error_and_right_distance():
    origin = (-2.0, -6.0, 0.0)
    t = np.linspace(0, 10, 101)
    truth = np.column_stack((t, -2.0 + 0.5 * t, np.full_like(t, -6.0)))  # 5 m east
    slam = np.column_stack((t, 0.5 * t, np.zeros_like(t)))
    r = position_errors(slam, truth, origin)
    assert r['rmse'] == pytest.approx(0, abs=1e-12)
    assert r['final'] == pytest.approx(0, abs=1e-12)
    assert r['distance'] == pytest.approx(5.0)
    assert r['t'][0] == 0 and r['t'][-1] == pytest.approx(10)


def test_constant_offset_and_interpolation():
    origin = (0.0, 0.0, 0.0)
    # Truth at 1 Hz, SLAM at 10 Hz and shifted 3 cm north: truth must be
    # interpolated, and every error is exactly 3 cm.
    tt = np.arange(0, 11, 1.0)
    truth = np.column_stack((tt, tt, np.zeros_like(tt)))
    ts = np.arange(0, 10.01, 0.1)
    slam = np.column_stack((ts, ts, np.full_like(ts, 0.03)))
    r = position_errors(slam, truth, origin)
    assert r['err'] == pytest.approx(np.full(len(ts), 0.03))
    assert r['rmse'] == pytest.approx(0.03)
    assert r['max'] == pytest.approx(0.03)


def test_unsorted_input_is_sorted_by_time():
    origin = (0.0, 0.0, 0.0)
    truth = np.array([[2, 2, 0], [0, 0, 0], [1, 1, 0]], dtype=float)
    slam = np.array([[1, 1, 0], [2, 2.1, 0], [0, 0, 0]], dtype=float)
    r = position_errors(slam, truth, origin)
    assert r['final'] == pytest.approx(0.1)
    assert r['err'][0] == pytest.approx(0.0)
