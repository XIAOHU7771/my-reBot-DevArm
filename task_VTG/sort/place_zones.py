"""
task_VTG/sort/place_zones.py
============================
M3：分拣放置区查询与进区判定。
"""

from __future__ import annotations

import numpy as np

from config import ZONE_ACCEPT_M, zone_xy_for_kind


def place_target_xy(kind: str) -> np.ndarray:
    """返回放置区中心 XY。"""
    xy = zone_xy_for_kind(kind)
    return np.array([xy[0], xy[1]], dtype=float)


def in_zone(pos_xyz, kind: str, tol_m: float = ZONE_ACCEPT_M) -> bool:
    """物体最终位置是否落入对应放置区（仅判 XY）。"""
    p = np.asarray(pos_xyz, dtype=float).reshape(-1)
    c = place_target_xy(kind)
    return float(np.linalg.norm(p[:2] - c)) < float(tol_m)


def zone_name(kind: str) -> str:
    return "ZONE_A_SOFT" if kind == "soft" else "ZONE_B_HARD"
