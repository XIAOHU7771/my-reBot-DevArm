"""
task_VTG/scripts/test_detect_localize.py
========================================
M2 入口：红/黄检测 → 基座系坐标，与 getBasePosition 比误差。

运行（仓库根目录）:
  python task_VTG/scripts/test_detect_localize.py --direct
  python task_VTG/scripts/test_detect_localize.py
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_VTG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_B2_ROOT = os.path.join(_REPO_ROOT, "task_B2")
for _p in (_REPO_ROOT, _VTG_ROOT, _B2_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from utils.robot import load_robot  # noqa: E402

from vision.camera import CameraConfig, capture_rgbd, save_rgb  # noqa: E402
from vision.detect import (  # noqa: E402
    detect_colored_cubes,
    draw_detections_overlay,
)
from vision.handeye import cam_to_base, sanity_check_eye  # noqa: E402

# 与 M1 一致的固定位姿（本里程碑不做随机）
_CUBE_SPECS = (
    ("red", [0.35, -0.10, 0.025], [0.85, 0.1, 0.1, 1.0]),
    ("yellow", [0.35, 0.10, 0.025], [0.95, 0.85, 0.2, 1.0]),
)

_XY_ERR_LIMIT_CM = 2.0


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M2: 颜色检测 + 手眼定位误差")
    ap.add_argument("--direct", action="store_true", help="无 GUI")
    ap.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="输出目录（默认 task_VTG/debug_images）",
    )
    return ap.parse_args()


def setup_scene(direct: bool) -> dict[str, int]:
    """地面 + 臂 + 固定红/黄块，返回 label→bodyId。"""
    if direct:
        cid = p.connect(p.DIRECT)
    else:
        cid = p.connect(p.GUI, options="--width=960 --height=720")
    if cid < 0:
        raise RuntimeError("无法连接 PyBullet")

    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    load_robot()

    bodies: dict[str, int] = {}
    for label, pos, rgba in _CUBE_SPECS:
        body = p.loadURDF("cube_small.urdf", basePosition=pos, useFixedBase=True)
        p.changeVisualShape(body, -1, rgbaColor=rgba)
        bodies[label] = body

    for _ in range(10):
        p.stepSimulation()
        if not direct:
            time.sleep(1.0 / 240.0)

    return bodies


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir or os.path.join(_VTG_ROOT, "debug_images")
    os.makedirs(out_dir, exist_ok=True)

    cfg = CameraConfig()
    assert sanity_check_eye(cfg), "handeye 自检失败：光心 ↔ cfg.eye"

    bodies = setup_scene(direct=bool(args.direct))
    frame = capture_rgbd(cfg)
    dets = detect_colored_cubes(frame.rgb, frame.depth_m, cfg)

    xyz_base_vis: dict[str, np.ndarray] = {}
    rows: list[tuple] = []
    for det in dets:
        xyz_b = cam_to_base(det.xyz_cam, cfg)
        xyz_base_vis[det.label] = xyz_b
        if det.label not in bodies:
            continue
        gt, _ = p.getBasePositionAndOrientation(bodies[det.label])
        gt = np.asarray(gt, dtype=np.float64)
        err = xyz_b - gt
        err_xy_cm = float(np.linalg.norm(err[:2]) * 100.0)
        err_z_cm = float(abs(err[2]) * 100.0)
        rows.append((det.label, xyz_b, gt, err_xy_cm, err_z_cm))

    overlay = draw_detections_overlay(frame.rgb, dets, xyz_base_vis)
    overlay_path = save_rgb(os.path.join(out_dir, "detect_overlay.png"), overlay)

    print("========== M2 检测 + 手眼 ==========")
    print(f"连接模式: {'DIRECT' if args.direct else 'GUI'}")
    print(f"检出数量: {len(dets)}  labels={[d.label for d in dets]}")
    print(
        f"{'label':8s} | {'xyz_vis':28s} | {'xyz_gt':28s} | "
        f"{'err_xy_cm':10s} | {'err_z_cm':8s}"
    )
    print("-" * 100)
    ok = True
    if len(rows) < 2:
        ok = False
        print("!! 未同时检出红与黄，验收失败")
    for label, xyz_b, gt, err_xy_cm, err_z_cm in rows:
        print(
            f"{label:8s} | "
            f"({xyz_b[0]:7.4f},{xyz_b[1]:7.4f},{xyz_b[2]:7.4f}) | "
            f"({gt[0]:7.4f},{gt[1]:7.4f},{gt[2]:7.4f}) | "
            f"{err_xy_cm:10.2f} | {err_z_cm:8.2f}"
        )
        if err_xy_cm >= _XY_ERR_LIMIT_CM:
            ok = False
    print("-" * 100)
    print(f"标注图: {overlay_path}")
    print(
        f"验收 XY < {_XY_ERR_LIMIT_CM:.1f} cm: "
        f"{'PASS' if ok else 'FAIL'}"
    )
    print("====================================")

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

    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
