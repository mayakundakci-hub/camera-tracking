#!/usr/bin/env python3
"""
compute_transform.py

"""

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np


def load_points(csv_path: str):
    fanuc_pts, opti_pts = [], []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        required = {"fanuc_x", "fanuc_y", "fanuc_z", "opti_x", "opti_y", "opti_z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            sys.exit(f"CSV missing columns: {missing}. "
                     f"Expected: fanuc_x,fanuc_y,fanuc_z,opti_x,opti_y,opti_z")
        for row in reader:
            fanuc_pts.append([float(row["fanuc_x"]), float(row["fanuc_y"]), float(row["fanuc_z"])])
            opti_pts.append([float(row["opti_x"]), float(row["opti_y"]), float(row["opti_z"])])

    fanuc_pts = np.array(fanuc_pts)
    opti_pts = np.array(opti_pts)

    if len(fanuc_pts) < 3:
        sys.exit(f"Need at least 3 point correspondences, got {len(fanuc_pts)}")
    if _is_collinear(opti_pts):
        sys.exit("OptiTrack points appear collinear. "
                 "Pick points that span 3D space (not all on one line).")

    return fanuc_pts, opti_pts


def _is_collinear(pts: np.ndarray, tol: float = 1e-6) -> bool:
    if len(pts) < 3:
        return True
    v1 = pts[1] - pts[0]
    for p in pts[2:]:
        v2 = p - pts[0]
        if np.linalg.norm(np.cross(v1, v2)) > tol:
            return False
    return True


def kabsch(opti_pts: np.ndarray, fanuc_pts: np.ndarray):
    """
    Solve for R, T such that: fanuc_pts ~= R @ opti_pts.T + T
    i.e. B = R*A + T, where A = opti_pts, B = fanuc_pts (matches the
    naming convention in the acupuncture paper's eq. B = RA + T).

    Returns R (3x3), T (3,), residuals (per-point distance after
    applying the transform, same units as your input points).
    """
    A = opti_pts
    B = fanuc_pts

    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)

    A_c = A - centroid_A
    B_c = B - centroid_B

    # E = (A - centroidA)(B - centroidB)^T  -> SVD -> R = V U^T
    H = A_c.T @ B_c
    U, S, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    correction = np.diag([1, 1, d])   # guards against a reflection instead of rotation
    R = Vt.T @ correction @ U.T

    T = centroid_B - R @ centroid_A

    # Residuals: apply transform to A, compare against B
    A_transformed = (R @ A.T).T + T
    residuals = np.linalg.norm(A_transformed - B, axis=1)

    return R, T, residuals


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv_path", help="CSV of point correspondences (see script docstring for format)")
    ap.add_argument("--out", default="calibration/transform.json", help="Output path for transform.json")
    ap.add_argument("--residual-units-are-mm", action="store_true",
                     help="Set if your input CSV points are already in mm (skips the meters->mm conversion for residual reporting)")
    args = ap.parse_args()

    fanuc_pts, opti_pts = load_points(args.csv_path)
    R, T, residuals = kabsch(opti_pts, fanuc_pts)

    residuals_mm = residuals if args.residual_units_are_mm else residuals * 1000.0
    mean_residual_mm = float(np.mean(residuals_mm))
    max_residual_mm = float(np.max(residuals_mm))

    print(f"Used {len(fanuc_pts)} point correspondences")
    print(f"Per-point residuals (mm): {np.round(residuals_mm, 3).tolist()}")
    print(f"Mean residual: {mean_residual_mm:.3f} mm   Max residual: {max_residual_mm:.3f} mm")
    if mean_residual_mm > 5.0:
        print("WARNING: residual is fairly large — check for a mismatched point pair, "
              "unit mismatch (mm vs m), or a marker offset error before trusting this transform.")

    out = {
        "_comment": "Rigid transform optitrack_world -> fanuc_base, computed via Kabsch/SVD point registration.",
        "rotation": R.tolist(),
        "translation_m": T.tolist(),
        "registration_residual_mm": mean_residual_mm,
        "registration_residual_max_mm": max_residual_mm,
        "num_correspondence_points": len(fanuc_pts),
    }

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(out, indent=2))
    print(f"\nWrote {out_path}")


if __name__ == "__main__":
    main()
