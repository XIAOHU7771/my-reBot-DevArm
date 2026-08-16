"""
task_VTG/vision/camera.py
=========================
M1：Eye-to-Hand 顶视 RGB-D 封装。

说明：
  - 相机固定在场景上方看工作台（不装在手腕上）。
  - 本模块只负责取图与存图；不做检测、手眼变换、抓取。
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any

import numpy as np
import pybullet as p


@dataclass
class CameraConfig:
    """Eye-to-Hand 顶视相机参数（仿真内已知位姿）。"""

    width: int = 640
    height: int = 480
    fov_deg: float = 60.0
    near: float = 0.05
    far: float = 2.0
    # 光心：工作台上方（与 B2 物体大致 x≈0.32~0.38 对齐）
    eye: tuple[float, float, float] = (0.35, 0.0, 0.85)
    # 看向台面中心附近
    target: tuple[float, float, float] = (0.35, 0.0, 0.0)
    up: tuple[float, float, float] = (0.0, 1.0, 0.0)


@dataclass
class CameraFrame:
    """一帧 RGB-D。"""

    rgb: Any  # HxWx3 uint8
    depth_m: Any  # HxW float，米
    width: int
    height: int
    config: CameraConfig


def build_view_matrix(cfg: CameraConfig) -> list[float]:
    """由 eye/target/up 生成 PyBullet viewMatrix。"""
    return p.computeViewMatrix(
        cameraEyePosition=list(cfg.eye),
        cameraTargetPosition=list(cfg.target),
        cameraUpVector=list(cfg.up),
    )


def build_projection_matrix(cfg: CameraConfig) -> list[float]:
    """由 fov/aspect/near/far 生成 projectionMatrix。"""
    aspect = float(cfg.width) / float(cfg.height)
    return p.computeProjectionMatrixFOV(
        fov=float(cfg.fov_deg),
        aspect=aspect,
        nearVal=float(cfg.near),
        farVal=float(cfg.far),
    )


def intrinsics_from_config(cfg: CameraConfig) -> tuple[float, float, float, float]:
    """
    由垂直 FOV 推导针孔内参 (fx, fy, cx, cy)。
    与 computeProjectionMatrixFOV 一致：fx == fy。
    """
    fov_rad = np.deg2rad(float(cfg.fov_deg))
    fy = (0.5 * float(cfg.height)) / np.tan(0.5 * fov_rad)
    fx = fy
    cx = 0.5 * float(cfg.width)
    cy = 0.5 * float(cfg.height)
    return float(fx), float(fy), float(cx), float(cy)


def _depth_buffer_to_meters(depth_buffer: np.ndarray, near: float, far: float) -> np.ndarray:
    """
    PyBullet depth buffer ∈ [0,1] → 真实距离（米）。
    公式见 PyBullet Quickstart：线性化非线性深度缓冲。
    """
    depth_buffer = np.asarray(depth_buffer, dtype=np.float64)
    # 避免除零
    denom = far - (far - near) * depth_buffer
    denom = np.clip(denom, 1e-8, None)
    return (far * near) / denom


def capture_rgbd(cfg: CameraConfig | None = None) -> CameraFrame:
    """
    调用 p.getCameraImage，返回 RGB 与深度（米）。
    调用前须已 connect 且场景已加载。
    """
    if cfg is None:
        cfg = CameraConfig()

    view = build_view_matrix(cfg)
    proj = build_projection_matrix(cfg)

    # DIRECT 下用 TinyRenderer 更稳；GUI 可用硬件 OpenGL
    try:
        info = p.getConnectionInfo()
        method = int(info.get("connectionMethod", -1))
    except Exception:
        method = -1
    renderer = (
        p.ER_BULLET_HARDWARE_OPENGL if method == int(p.GUI) else p.ER_TINY_RENDERER
    )

    w, h = int(cfg.width), int(cfg.height)
    _, _, rgb_buf, depth_buf, _ = p.getCameraImage(
        width=w,
        height=h,
        viewMatrix=view,
        projectionMatrix=proj,
        renderer=renderer,
    )

    rgb = np.asarray(rgb_buf, dtype=np.uint8).reshape(h, w, 4)[:, :, :3].copy()
    depth_buf = np.asarray(depth_buf, dtype=np.float32).reshape(h, w)
    depth_m = _depth_buffer_to_meters(depth_buf, cfg.near, cfg.far).astype(np.float32)

    return CameraFrame(
        rgb=rgb,
        depth_m=depth_m,
        width=w,
        height=h,
        config=cfg,
    )


def depth_to_vis_uint8(depth_m: Any, near: float, far: float) -> np.ndarray:
    """深度（米）→ HxW uint8，近亮远暗，便于存 PNG。"""
    d = np.asarray(depth_m, dtype=np.float64)
    d = np.clip(d, near, far)
    # 归一化到 0~1 再反相：近处更亮
    norm = (d - near) / max(far - near, 1e-8)
    vis = ((1.0 - norm) * 255.0).astype(np.uint8)
    return vis


def save_rgb(path: str, rgb: Any) -> str:
    """保存 RGB 图，返回路径。"""
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    import matplotlib.image as mpimg

    mpimg.imsave(path, np.asarray(rgb, dtype=np.uint8))
    return path


def save_depth_vis(path: str, depth_m: Any, near: float, far: float) -> str:
    """保存深度可视化图，返回路径。"""
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    import matplotlib.image as mpimg

    vis = depth_to_vis_uint8(depth_m, near, far)
    # 存成灰度三通道，兼容部分看图软件
    vis3 = np.stack([vis, vis, vis], axis=-1)
    mpimg.imsave(path, vis3)
    return path
