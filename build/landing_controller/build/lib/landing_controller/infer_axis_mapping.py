#!/usr/bin/env python3
"""
Infer camera->ENU axis signs/permutation from a short recorded sequence.

Expected CSV columns:
  - cam_x, cam_y, cam_z
And either:
  A) ref_east, ref_north, ref_up
     (continuous reference signals), OR
  B) segment
     (discrete labels from this set: E+, E-, N+, N-, U+, U-)

This tool tries all 48 signed-permutation mappings and picks the one that
maximizes correlation to your reference motion.
"""

from __future__ import annotations

import argparse
import csv
import itertools
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np


SEGMENT_TO_REF: Dict[str, Tuple[float, float, float]] = {
    "E+": (1.0, 0.0, 0.0),
    "E-": (-1.0, 0.0, 0.0),
    "N+": (0.0, 1.0, 0.0),
    "N-": (0.0, -1.0, 0.0),
    "U+": (0.0, 0.0, 1.0),
    "U-": (0.0, 0.0, -1.0),
}


@dataclass
class MappingResult:
    matrix: np.ndarray
    perm: Tuple[int, int, int]
    signs: Tuple[int, int, int]
    score: float
    corr: np.ndarray


def _corr(a: np.ndarray, b: np.ndarray) -> float:
    a_std = float(np.std(a))
    b_std = float(np.std(b))
    if a_std < 1e-12 or b_std < 1e-12:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


def _axis_name(idx: int) -> str:
    return ("cam_x", "cam_y", "cam_z")[idx]


def _sign_str(v: int) -> str:
    return "-" if v < 0 else "+"


def read_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    with path.open("r", newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError("CSV is empty.")

    for col in ("cam_x", "cam_y", "cam_z"):
        if col not in rows[0]:
            raise ValueError(f"Missing required column: {col}")

    cam = np.array(
        [[float(r["cam_x"]), float(r["cam_y"]), float(r["cam_z"])] for r in rows],
        dtype=float,
    )

    has_ref = all(k in rows[0] for k in ("ref_east", "ref_north", "ref_up"))
    has_segment = "segment" in rows[0]

    if has_ref:
        ref = np.array(
            [
                [float(r["ref_east"]), float(r["ref_north"]), float(r["ref_up"])]
                for r in rows
            ],
            dtype=float,
        )
    elif has_segment:
        ref_list: List[Tuple[float, float, float]] = []
        for i, r in enumerate(rows, start=1):
            label = r["segment"].strip().upper()
            if label not in SEGMENT_TO_REF:
                valid = ", ".join(SEGMENT_TO_REF.keys())
                raise ValueError(
                    f"Invalid segment label '{label}' on row {i}. Valid: {valid}"
                )
            ref_list.append(SEGMENT_TO_REF[label])
        ref = np.array(ref_list, dtype=float)
    else:
        raise ValueError(
            "Need either (ref_east, ref_north, ref_up) columns or a 'segment' column."
        )

    return cam, ref


def infer_mapping(cam: np.ndarray, ref: np.ndarray) -> Tuple[MappingResult, MappingResult]:
    # Remove constant offsets for robust correlation scoring.
    cam0 = cam - np.mean(cam, axis=0, keepdims=True)
    ref0 = ref - np.mean(ref, axis=0, keepdims=True)

    results: List[MappingResult] = []

    for perm in itertools.permutations((0, 1, 2)):  # output axis -> input axis
        for signs in itertools.product((-1, 1), repeat=3):
            m = np.zeros((3, 3), dtype=float)
            for out_axis, in_axis in enumerate(perm):
                m[out_axis, in_axis] = float(signs[out_axis])

            pred = cam0 @ m.T
            corr = np.array([_corr(pred[:, i], ref0[:, i]) for i in range(3)], dtype=float)
            score = float(np.mean(corr))
            results.append(MappingResult(m, perm, signs, score, corr))

    results.sort(key=lambda r: r.score, reverse=True)
    return results[0], results[1]


def print_result(best: MappingResult, second: MappingResult) -> None:
    print("Best mapping found")
    print("------------------")
    print(f"score        : {best.score:.4f}")
    print(f"axis corr    : east={best.corr[0]:.4f}, north={best.corr[1]:.4f}, up={best.corr[2]:.4f}")
    print(f"confidence   : {best.score - second.score:.4f} (best - second)")
    print()
    print("Mapping equations")
    print("-----------------")
    out_names = ("east", "north", "up")
    for i, out_name in enumerate(out_names):
        in_axis = best.perm[i]
        sign = _sign_str(best.signs[i])
        print(f"{out_name:>5s} = {sign}{_axis_name(in_axis)}")
    print()
    print("Patch hint for camera_to_enu(cam_xyz):")
    print("--------------------------------------")
    for i, out_name in enumerate(out_names):
        in_axis = best.perm[i]
        sign = "-" if best.signs[i] < 0 else ""
        var = ("cx", "cy", "cz")[in_axis]
        print(f"    {out_name} = {sign}{var}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Infer best camera->ENU signed axis mapping from CSV."
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        help="CSV path with cam_x,cam_y,cam_z and either ref_* or segment labels.",
    )
    args = parser.parse_args()

    cam, ref = read_csv(args.csv_path)
    best, second = infer_mapping(cam, ref)
    print_result(best, second)


if __name__ == "__main__":
    main()
