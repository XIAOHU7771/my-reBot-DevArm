"""
task_VTG/tracking/retrack.py
============================
M6：位置级重跟踪 — 检测当前颜色目标，位移超阈值则更新 XY。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from config import CUBE_HALF, RETRACK_XY_THRESH_M
from vision.camera import CameraConfig, capture_rgbd
from vision.detect import detect_colored_cubes
from vision.handeye import cam_to_base


@dataclass
class RetrackResult:
    triggered: bool
    xy_old: tuple[float, float]
    xy_new: tuple[float, float] | None
    err_cm: float
    reason: str  # "" | "moved" | "detect_miss" | "below_thresh"


def maybe_retrack(
    color: str,
    xy_goal: tuple[float, float],
    camera_cfg: CameraConfig | None = None,
    *,
    thresh_m: float = RETRACK_XY_THRESH_M,
    frame: Any = None,
) -> RetrackResult:
    """
    拍一帧检测 `color`；若与 xy_goal 的 XY 差 > thresh_m 则 triggered=True。
    """
    if camera_cfg is None:
        camera_cfg = CameraConfig()
    if frame is None:
        frame = capture_rgbd(camera_cfg)

    xy_old = (float(xy_goal[0]), float(xy_goal[1]))
    dets = detect_colored_cubes(frame.rgb, frame.depth_m, camera_cfg)
    match = next((d for d in dets if d.label == color), None)
    if match is None:
        return RetrackResult(
            triggered=False,
            xy_old=xy_old,
            xy_new=None,
            err_cm=0.0,
            reason="detect_miss",
        )

    xyz = cam_to_base(match.xyz_cam, camera_cfg)
    xy_new = (float(xyz[0]), float(xyz[1]))
    err_m = float(np.linalg.norm(np.array(xy_new) - np.array(xy_old)))
    err_cm = err_m * 100.0
    if err_m < float(thresh_m):
        return RetrackResult(
            triggered=False,
            xy_old=xy_old,
            xy_new=xy_new,
            err_cm=err_cm,
            reason="below_thresh",
        )
    return RetrackResult(
        triggered=True,
        xy_old=xy_old,
        xy_new=xy_new,
        err_cm=err_cm,
        reason="moved",
    )


def log_retrack(rr: RetrackResult) -> None:
    """统一 [RETRACK] 日志格式。"""
    if rr.triggered and rr.xy_new is not None:
        print(
            f"  [RETRACK] moved  err={rr.err_cm:.2f}cm  "
            f"old=({rr.xy_old[0]:.3f},{rr.xy_old[1]:.3f})  "
            f"new=({rr.xy_new[0]:.3f},{rr.xy_new[1]:.3f})"
        )
    elif rr.reason == "detect_miss":
        print(f"  [RETRACK] detect_miss  keep old=({rr.xy_old[0]:.3f},{rr.xy_old[1]:.3f})")
    else:
        print(
            f"  [RETRACK] below_thresh  err={rr.err_cm:.2f}cm  "
            f"goal=({rr.xy_old[0]:.3f},{rr.xy_old[1]:.3f})"
        )


def grasp_xy_from_goal(xy: tuple[float, float]) -> np.ndarray:
    """视觉/重跟踪 XY → 抓取用桌面 3D。"""
    return np.array([float(xy[0]), float(xy[1]), float(CUBE_HALF)], dtype=float)
