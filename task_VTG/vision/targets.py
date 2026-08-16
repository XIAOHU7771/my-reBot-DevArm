"""
task_VTG/vision/targets.py
==========================
M4：RGB-D 检测 → 流水线待抓目标（视觉 XY + 颜色类别）。

约定：
  - 红 → hard / 铁块；黄 → soft / 海绵
  - 抓取只用视觉 XY；Z 固定为 CUBE_HALF（顶面深度不当地板高度）
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from config import COLOR_META, CUBE_HALF
from vision.camera import CameraConfig, CameraFrame, capture_rgbd
from vision.detect import detect_colored_cubes
from vision.handeye import cam_to_base


@dataclass
class VisionTarget:
    """单个视觉目标（可直接喂给 pipeline）。"""

    id: str
    label: str
    kind: str
    soft: bool
    mass: float
    xy: tuple[float, float]
    xyz_base: np.ndarray
    color: str
    u: float = 0.0
    v: float = 0.0
    body_id: int | None = None

    def as_object_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "label": self.label,
            "kind": self.kind,
            "soft": self.soft,
            "mass": self.mass,
            "xy": self.xy,
            "color": self.color,
            "xyz_base": self.xyz_base,
            "u": self.u,
            "v": self.v,
        }


def detect_targets(
    camera_cfg: CameraConfig | None = None,
    bodies_by_color: dict[str, int] | None = None,
    frame: CameraFrame | None = None,
) -> list[VisionTarget]:
    """
    拍一帧（或用给定 frame）→ 红/黄检测 → 基座系目标列表。
    bodies_by_color: {"red": bodyId, "yellow": bodyId} 可选绑定。
    """
    if camera_cfg is None:
        camera_cfg = CameraConfig()
    if frame is None:
        frame = capture_rgbd(camera_cfg)

    dets = detect_colored_cubes(frame.rgb, frame.depth_m, camera_cfg)
    out: list[VisionTarget] = []
    for det in dets:
        meta = COLOR_META.get(det.label)
        if meta is None:
            continue
        xyz = cam_to_base(det.xyz_cam, camera_cfg)
        xy = (float(xyz[0]), float(xyz[1]))
        body = None
        if bodies_by_color is not None:
            body = bodies_by_color.get(det.label)
        out.append(
            VisionTarget(
                id=str(meta["id"]),
                label=str(meta["label"]),
                kind=str(meta["kind"]),
                soft=bool(meta["soft"]),
                mass=float(meta["mass"]),
                xy=xy,
                xyz_base=xyz,
                color=det.label,
                u=float(det.u),
                v=float(det.v),
                body_id=body,
            )
        )
    # 稳定顺序：hard 先、soft 后（与 M3 一致）
    out.sort(key=lambda t: 0 if t.kind == "hard" else 1)
    return out


def detect_targets_with_frame(
    camera_cfg: CameraConfig | None = None,
    bodies_by_color: dict[str, int] | None = None,
) -> tuple[CameraFrame, list[VisionTarget]]:
    """拍一帧并检测，同时返回 frame（供 M5 标注）。"""
    if camera_cfg is None:
        camera_cfg = CameraConfig()
    frame = capture_rgbd(camera_cfg)
    return frame, detect_targets(camera_cfg, bodies_by_color=bodies_by_color, frame=frame)


def targets_to_objects(targets: list[VisionTarget]) -> list[dict[str, Any]]:
    """VisionTarget → pipeline objects 字典列表。"""
    return [t.as_object_dict() for t in targets]


def compare_to_gt(
    targets: list[VisionTarget],
    gt_xy_by_color: dict[str, tuple[float, float]],
) -> list[dict[str, Any]]:
    """打印/返回视觉 vs GT 的 XY 误差（cm）。"""
    rows = []
    for t in targets:
        gt = gt_xy_by_color.get(t.color)
        if gt is None:
            continue
        err = np.linalg.norm(np.array(t.xy) - np.array(gt)) * 100.0
        rows.append(
            {
                "color": t.color,
                "xy_vis": t.xy,
                "xy_gt": gt,
                "err_xy_cm": float(err),
                "z_vis": float(t.xyz_base[2]),
                "z_grasp": float(CUBE_HALF),
            }
        )
    return rows
