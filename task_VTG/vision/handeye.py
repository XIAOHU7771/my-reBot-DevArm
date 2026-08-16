"""
task_VTG/vision/handeye.py
==========================
M2：相机系 → 基座系（Eye-to-Hand，外参由仿真已知相机位姿给出）。

约定：
  - 机械臂固定在世界原点 → 世界系 ≈ 基座系。
  - T_base_cam：把相机系点变到基座系：p_base = R @ p_cam + t。
  - 相机系与 PyBullet viewMatrix / OpenGL 一致（看向 -Z）。
"""

from __future__ import annotations

import numpy as np

from vision.camera import CameraConfig, build_view_matrix


def view_matrix_4x4(cfg: CameraConfig) -> np.ndarray:
    """
    PyBullet computeViewMatrix → 4x4（列主序 → ndarray）。
    语义：p_cam_h = V @ p_world_h（齐次列向量）。
    """
    vm = build_view_matrix(cfg)
    return np.asarray(vm, dtype=np.float64).reshape(4, 4, order="F")


def camera_pose_from_config(cfg: CameraConfig) -> tuple[np.ndarray, np.ndarray]:
    """
    由 CameraConfig 得到 T_base_cam 的 R(3x3)、t(3,)。

    因 V 把 world→cam，故 cam→world(=base) 为 V^{-1}：
      [R t] = V^{-1} 的左上 3x3 与平移列。
    """
    v = view_matrix_4x4(cfg)
    t_cam_base = np.linalg.inv(v)
    R = t_cam_base[:3, :3].copy()
    t = t_cam_base[:3, 3].copy()
    return R, t


def cam_to_base(xyz_cam: np.ndarray, cfg: CameraConfig) -> np.ndarray:
    """
    (X,Y,Z)_cam → (x,y,z)_base。

    p_base = R @ p_cam + t，其中 (R,t) = T_base_cam。
    """
    R, t = camera_pose_from_config(cfg)
    p_cam = np.asarray(xyz_cam, dtype=np.float64).reshape(3)
    return R @ p_cam + t


def cam_to_base_batch(xyz_cam: np.ndarray, cfg: CameraConfig) -> np.ndarray:
    """Nx3 批量变换。"""
    R, t = camera_pose_from_config(cfg)
    pts = np.asarray(xyz_cam, dtype=np.float64).reshape(-1, 3)
    return (pts @ R.T) + t


def sanity_check_eye(cfg: CameraConfig, atol: float = 1e-5) -> bool:
    """
    自检：相机光心在相机系为原点，变到 base 应接近 cfg.eye。
    （不依赖 PyBullet 已连接，仅用矩阵。）
    """
    origin_cam = np.zeros(3, dtype=np.float64)
    eye_est = cam_to_base(origin_cam, cfg)
    return bool(np.allclose(eye_est, np.asarray(cfg.eye, dtype=np.float64), atol=atol))
