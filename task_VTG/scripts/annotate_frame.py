"""
task_VTG/scripts/annotate_frame.py
==================================
M5 快路径：随机摆放 → 顶视检测 → 保存标注图（不跑抓取）。

运行（仓库根目录）:
  python task_VTG/scripts/annotate_frame.py --direct --seed 7
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

import matplotlib

matplotlib.use("Agg")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M5: 只标注不抓取")
    ap.add_argument("--direct", action="store_true", help="无 GUI")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument(
        "--out",
        type=str,
        default=None,
        help="输出 PNG（默认 debug_images/m5_annotate_seedN.png）",
    )
    return ap.parse_args()


def _away_from_zones(xy: np.ndarray, clear: float) -> bool:
    from config import ZONE_A_SOFT, ZONE_B_HARD

    for z in (ZONE_A_SOFT, ZONE_B_HARD):
        if float(np.linalg.norm(xy - np.asarray(z, dtype=float))) < clear:
            return False
    return True


def sample_spawn_xy(rng: np.random.Generator, existing: list[np.ndarray]) -> np.ndarray:
    from config import (
        SPAWN_MIN_SEP,
        SPAWN_X_RANGE,
        SPAWN_Y_RANGE,
        SPAWN_ZONE_CLEAR,
    )

    for _ in range(200):
        xy = np.array(
            [rng.uniform(*SPAWN_X_RANGE), rng.uniform(*SPAWN_Y_RANGE)],
            dtype=float,
        )
        if not _away_from_zones(xy, SPAWN_ZONE_CLEAR):
            continue
        if any(float(np.linalg.norm(xy - e)) < SPAWN_MIN_SEP for e in existing):
            continue
        return xy
    raise RuntimeError("随机摆放采样失败")


def main() -> None:
    args = parse_args()
    seed = int(args.seed)
    rng = np.random.default_rng(seed)

    from utils.robot import load_robot
    from utils.gripper import init_gripper
    from utils.ik import init_ik

    from config import COLOR_META, COLOR_RGBA, CUBE_HALF, HOME_POS
    from grasp.adaptive_bridge import import_b2_afc
    from vision.annotate import annotate_from_targets, save_annotation
    from vision.camera import CameraConfig
    from vision.targets import compare_to_gt, detect_targets_with_frame

    afc = import_b2_afc()
    afc.connect_pybullet(direct=bool(args.direct))
    p.setGravity(0, 0, -9.8)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")

    robot_data = load_robot()
    init_gripper(robot_data)
    init_ik(robot_data)

    xy_red = sample_spawn_xy(rng, [])
    xy_yel = sample_spawn_xy(rng, [xy_red])
    gt = {
        "red": (float(xy_red[0]), float(xy_red[1])),
        "yellow": (float(xy_yel[0]), float(xy_yel[1])),
    }

    iron = afc.load_iron_block(
        [gt["red"][0], gt["red"][1], CUBE_HALF],
        mass=float(COLOR_META["red"]["mass"]),
        pin=True,
    )
    sponge = afc.load_sponge(
        [gt["yellow"][0], gt["yellow"][1], CUBE_HALF],
        mass=float(COLOR_META["yellow"]["mass"]),
        pin=True,
    )
    p.changeVisualShape(iron, -1, rgbaColor=COLOR_RGBA["red"])
    p.changeVisualShape(sponge, -1, rgbaColor=COLOR_RGBA["yellow"])
    p.setCollisionFilterPair(iron, sponge, -1, -1, 0)

    # 回 HOME，减少臂遮挡
    home = np.array(HOME_POS, dtype=float)
    home_orn = p.getQuaternionFromEuler(afc.HOME_EULER)
    afc.open_gripper(robot_data["robot"])
    try:
        afc.move_tcp(
            robot_data["robot"],
            robot_data["arm_joints"],
            robot_data["ee_link"],
            home,
            home_orn,
            afc.OPEN,
            steps=120,
        )
    except RuntimeError as e:
        print(f"  [HOME] {e}")

    bodies_by_color = {"red": iron, "yellow": sponge}
    cfg = CameraConfig()
    frame, targets = detect_targets_with_frame(cfg, bodies_by_color=bodies_by_color)

    print("========== M5 标注 ==========")
    print(f"seed={seed}  detections={len(targets)}")
    for row in compare_to_gt(targets, gt):
        print(
            f"  {row['color']}: err_xy={row['err_xy_cm']:.2f}cm  "
            f"vis={np.round(row['xy_vis'], 4)}"
        )

    out = args.out or os.path.join(
        _VTG_ROOT, "debug_images", f"m5_annotate_seed{seed}.png"
    )
    if not targets:
        print("DETECT miss — 无标注图")
        p.disconnect()
        raise SystemExit(1)

    annot = annotate_from_targets(
        frame.rgb, targets, title=f"M5 annotate seed={seed}",
    )
    path = save_annotation(out, annot)
    print(f"已保存: {path}")
    print("============================")

    if not args.direct:
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(0.05)
        except Exception:
            pass
    p.disconnect()


if __name__ == "__main__":
    main()
