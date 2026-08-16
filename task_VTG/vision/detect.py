"""
task_VTG/vision/detect.py
=========================
M2：红/黄颜色检测 → 像素中心 + 深度 → 相机系 3D。

说明：
  - 仿真纯色块，用 HSV 阈值即可（不做深度学习）。
  - 仅依赖 NumPy（避免额外 OpenCV 安装）；算法与 cv2.inRange 等价。
  - 深度在 mask 内取中位数，减轻边缘飞点。
  - 相机系约定与 PyBullet/OpenGL 一致：X 右、Y 上、Z 朝相机后方（看向 -Z）。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from vision.camera import CameraConfig, intrinsics_from_config

# HSV 阈值（OpenCV 风格：H∈[0,180]，S/V∈[0,255]）
_HSV_RED_1 = ((0, 80, 60), (10, 255, 255))
_HSV_RED_2 = ((170, 80, 60), (180, 255, 255))
_HSV_YELLOW = ((18, 80, 60), (40, 255, 255))

_MIN_AREA_PX = 80


@dataclass
class Detection:
    """单个色块检测结果（相机系）。"""

    label: str  # "red" | "yellow"
    u: float
    v: float
    depth_m: float
    xyz_cam: np.ndarray  # (3,) OpenGL 相机系
    area_px: int


def rgb_to_hsv_opencv(rgb: np.ndarray) -> np.ndarray:
    """
    RGB uint8 → HSV uint8（H 0~180，与 OpenCV 一致，便于对照阈值文档）。
    """
    rgb_f = np.asarray(rgb, dtype=np.float64) / 255.0
    r, g, b = rgb_f[..., 0], rgb_f[..., 1], rgb_f[..., 2]
    maxc = np.maximum(np.maximum(r, g), b)
    minc = np.minimum(np.minimum(r, g), b)
    v = maxc
    span = maxc - minc
    s = np.where(maxc > 1e-8, span / np.maximum(maxc, 1e-8), 0.0)

    h = np.zeros_like(maxc)
    # R max
    mask = (maxc == r) & (span > 1e-8)
    h[mask] = ((g[mask] - b[mask]) / span[mask]) % 6.0
    # G max
    mask = (maxc == g) & (span > 1e-8)
    h[mask] = (b[mask] - r[mask]) / span[mask] + 2.0
    # B max
    mask = (maxc == b) & (span > 1e-8)
    h[mask] = (r[mask] - g[mask]) / span[mask] + 4.0
    h = (h * 30.0) % 180.0  # 60°→30 OpenCV units；再映射到 0~180

    hsv = np.stack(
        [
            np.clip(h, 0, 180),
            np.clip(s * 255.0, 0, 255),
            np.clip(v * 255.0, 0, 255),
        ],
        axis=-1,
    )
    return hsv.astype(np.uint8)


def _in_range(hsv: np.ndarray, lo: tuple[int, int, int], hi: tuple[int, int, int]) -> np.ndarray:
    lo_a = np.array(lo, dtype=np.uint8)
    hi_a = np.array(hi, dtype=np.uint8)
    return np.all((hsv >= lo_a) & (hsv <= hi_a), axis=-1)


def color_masks(rgb: Any) -> dict[str, np.ndarray]:
    """RGB → {label: bool mask}。"""
    hsv = rgb_to_hsv_opencv(np.asarray(rgb, dtype=np.uint8))
    red = _in_range(hsv, *_HSV_RED_1) | _in_range(hsv, *_HSV_RED_2)
    yellow = _in_range(hsv, *_HSV_YELLOW)
    return {"red": red, "yellow": yellow}


def _centroid_and_area(mask: np.ndarray) -> tuple[float, float, int] | None:
    ys, xs = np.where(mask)
    area = int(xs.size)
    if area < _MIN_AREA_PX:
        return None
    return float(xs.mean()), float(ys.mean()), area


def median_depth_in_mask(
    depth_m: Any, mask: np.ndarray, near: float, far: float
) -> float | None:
    d = np.asarray(depth_m, dtype=np.float64)
    sel = mask & np.isfinite(d) & (d > near) & (d < far)
    if not np.any(sel):
        return None
    return float(np.median(d[sel]))


def pixel_depth_to_cam(
    u: float,
    v: float,
    depth_m: float,
    cfg: CameraConfig,
) -> np.ndarray:
    """
    像素 + 光轴深度 → OpenGL 相机系 (X,Y,Z)。

    depth_m：正深度；相机看向 -Z，故 Z_cam = -depth_m；
    图像 v 向下而 Y_cam 向上，故 Y 取负。
    """
    fx, fy, cx, cy = intrinsics_from_config(cfg)
    x = (float(u) - cx) * float(depth_m) / fx
    y = -((float(v) - cy) * float(depth_m) / fy)
    z = -float(depth_m)
    return np.array([x, y, z], dtype=np.float64)


def detect_colored_cubes(
    rgb: Any,
    depth_m: Any,
    cfg: CameraConfig | None = None,
) -> list[Detection]:
    """检测红/黄块，返回相机系 Detection 列表。"""
    if cfg is None:
        cfg = CameraConfig()

    out: list[Detection] = []
    for label, mask in color_masks(rgb).items():
        stats = _centroid_and_area(mask)
        if stats is None:
            continue
        u, v, area = stats
        depth = median_depth_in_mask(depth_m, mask, cfg.near, cfg.far)
        if depth is None:
            continue
        out.append(
            Detection(
                label=label,
                u=u,
                v=v,
                depth_m=depth,
                xyz_cam=pixel_depth_to_cam(u, v, depth, cfg),
                area_px=area,
            )
        )
    return out


def draw_detections_overlay(
    rgb: Any,
    detections: list[Detection],
    xyz_base_by_label: dict[str, np.ndarray] | None = None,
) -> np.ndarray:
    """兼容旧接口：转交 vision.annotate。"""
    from vision.annotate import annotate_rgb

    items = []
    for det in detections:
        item = {"u": det.u, "v": det.v, "label": det.label, "color": det.label}
        if xyz_base_by_label and det.label in xyz_base_by_label:
            item["xyz_base"] = xyz_base_by_label[det.label]
        items.append(item)
    return annotate_rgb(rgb, items)
