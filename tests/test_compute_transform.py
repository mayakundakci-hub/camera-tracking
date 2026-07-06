"""
test_compute_transform.py

The Kabsch/SVD registration in calibration/compute_transform.py is the
highest-stakes math in this project: a silently wrong transform doesn't
crash anything — it just makes every error reading confidently wrong.
These tests exist to catch that category of bug specifically.

Run:
    pip install pytest numpy --break-system-packages
    pytest tests/test_compute_transform.py -v
"""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "calibration"))
from compute_transform import kabsch, _is_collinear  # noqa: E402


def make_correspondence(theta, translation, n_points=6, noise_std=0.0, seed=0):
    """Build a synthetic (opti_pts, fanuc_pts) pair related by a known R, T."""
    R_true = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0, 0, 1],
    ])
    T_true = np.array(translation)

    rng = np.random.default_rng(seed)
    opti_pts = rng.uniform(-1, 1, size=(n_points, 3))
    fanuc_pts = (R_true @ opti_pts.T).T + T_true
    if noise_std:
        fanuc_pts = fanuc_pts + rng.normal(0, noise_std, fanuc_pts.shape)
    return opti_pts, fanuc_pts, R_true, T_true


def test_recovers_known_rotation_and_translation_exactly_when_noise_free():
    opti_pts, fanuc_pts, R_true, T_true = make_correspondence(
        theta=0.3, translation=[1.2, -0.5, 0.1], noise_std=0.0)

    R, T, residuals = kabsch(opti_pts, fanuc_pts)

    np.testing.assert_allclose(R, R_true, atol=1e-8)
    np.testing.assert_allclose(T, T_true, atol=1e-8)
    np.testing.assert_allclose(residuals, 0.0, atol=1e-8)


def test_residual_reflects_injected_noise_magnitude():
    opti_pts, fanuc_pts, _, _ = make_correspondence(
        theta=0.7, translation=[0.0, 0.0, 0.5], n_points=8, noise_std=0.001)  # 1mm-scale noise

    _, _, residuals = kabsch(opti_pts, fanuc_pts)

    # Residual should be small and in the same ballpark as the injected noise,
    # not zero (noise exists) and not huge (transform is still basically right).
    assert 0.0 < residuals.mean() < 0.01


def test_identity_transform_when_frames_already_aligned():
    opti_pts, fanuc_pts, _, _ = make_correspondence(theta=0.0, translation=[0, 0, 0])

    R, T, residuals = kabsch(opti_pts, fanuc_pts)

    np.testing.assert_allclose(R, np.eye(3), atol=1e-8)
    np.testing.assert_allclose(T, [0, 0, 0], atol=1e-8)


def test_rejects_reflection_not_just_rotation():
    """
    A naive SVD solution (R = V U^T without the determinant-sign
    correction) can produce a reflection instead of a proper rotation
    when points are close to a degenerate configuration. det(R) must
    always be +1, never -1.
    """
    opti_pts, fanuc_pts, _, _ = make_correspondence(
        theta=1.9, translation=[3.0, -2.0, 0.4], n_points=4, seed=7)

    R, _, _ = kabsch(opti_pts, fanuc_pts)

    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-6)


def test_collinear_points_are_detected():
    collinear = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]], dtype=float)
    assert _is_collinear(collinear) is True


def test_non_collinear_points_are_not_flagged():
    spread = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float)
    assert _is_collinear(spread) is False


def test_two_points_are_trivially_collinear():
    # Can't determine a plane from 2 points; treated as degenerate/collinear
    assert _is_collinear(np.array([[0, 0, 0], [1, 1, 1]], dtype=float)) is True
