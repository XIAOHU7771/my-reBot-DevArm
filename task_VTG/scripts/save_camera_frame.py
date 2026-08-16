"""
task_VTG/scripts/save_camera_frame.py
=====================================
M1 入口：最小场景 + Eye-to-Hand 顶视拍一帧并保存。

运行（仓库根目录）:
  python task_VTG/scripts/save_camera_frame.py
  python task_VTG/scripts/save_camera_frame.py --direct
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data

# 保证可 import task_VTG 与 task_B2.utils
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_VTG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_B2_ROOT = os.path.join(_REPO_ROOT, "task_B2")
for _p in (_REPO_ROOT, _VTG_ROOT, _B2_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from utils.robot import load_robot  # noqa: E402  (from task_B2 on sys.path)

from vision.camera import (  # noqa: E402
    CameraConfig,
    capture_rgbd,
    save_depth_vis,
    save_rgb,
)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M1: 保存顶视 RGB-D 一帧")
    ap.add_argument("--direct", action="store_true", help="无 GUI（TinyRenderer）")
    ap.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="输出目录（默认 task_VTG/debug_images）",
    )
    return ap.parse_args()


def setup_minimal_scene(direct: bool) -> dict:
    """
    连接仿真 → 地面 → 加载机械臂。
    可选放两个静态 cube，仅方便画面里有物体（不做检测）。
    """
    if direct:
        cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.GUI, options="--width=960 --height=720")
    if cid < 0:
        raise RuntimeError("无法连接 PyBullet")

    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    # 只读复用 B2 加载逻辑；本步不启用力控业务
    robot_data = load_robot()

    # 静态参考物：红 / 黄各一，方便确认相机朝向工作台
    for pos, rgba in (
        ([0.35, -0.10, 0.025], [0.85, 0.1, 0.1, 1.0]),
        ([0.35, 0.10, 0.025], [0.95, 0.85, 0.2, 1.0]),
    ):
        body = p.loadURDF(
            "cube_small.urdf",
            basePosition=pos,
            useFixedBase=True,
        )
        p.changeVisualShape(body, -1, rgbaColor=rgba)

    # 稍步进，让渲染状态稳定
    for _ in range(10):
        p.stepSimulation()
        if not direct:
            time.sleep(1.0 / 240.0)

    return {"robot_data": robot_data, "direct": direct}


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir or os.path.join(_VTG_ROOT, "debug_images")
    os.makedirs(out_dir, exist_ok=True)

    setup_minimal_scene(direct=bool(args.direct))
    cfg = CameraConfig()
    frame = capture_rgbd(cfg)

    rgb_path = save_rgb(os.path.join(out_dir, "rgb.png"), frame.rgb)
    depth_path = save_depth_vis(
        os.path.join(out_dir, "depth.png"),
        frame.depth_m,
        near=cfg.near,
        far=cfg.far,
    )

    print("========== M1 相机存图 ==========")
    print(f"连接模式: {'DIRECT' if args.direct else 'GUI'}")
    print(f"RGB 尺寸: {frame.width}x{frame.height}")
    print(f"深度范围(米): min={float(np.nanmin(frame.depth_m)):.3f}  "
          f"max={float(np.nanmax(frame.depth_m)):.3f}")
    print(f"已保存 RGB : {rgb_path}")
    print(f"已保存 Depth: {depth_path}")
    print("=================================")

    if not args.direct:
        print("关闭 GUI 窗口或 Ctrl+C 结束...")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(0.05)
        except Exception:
            pass

    try:
        p.disconnect()
    except Exception:
        pass


if __name__ == "__main__":
    main()
