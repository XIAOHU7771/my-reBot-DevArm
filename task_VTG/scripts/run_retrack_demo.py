"""
task_VTG/scripts/run_retrack_demo.py
====================================
M6：单物体下探重跟踪演示（强制 nudge + 可选 retrack）。

运行（仓库根目录）:
  python task_VTG/scripts/run_retrack_demo.py --direct --retrack --nudge --no-force-log
  python task_VTG/scripts/run_retrack_demo.py --direct --nudge --no-force-log
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

if "--direct" in sys.argv:
    import matplotlib

    matplotlib.use("Agg")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M6: 下探重跟踪 demo")
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--retrack", action="store_true", help="开启视觉重跟踪")
    ap.add_argument("--nudge", action="store_true", help="预位后强制平移物体")
    ap.add_argument("--no-force-log", action="store_true")
    ap.add_argument(
        "--nudge-dy",
        type=float,
        default=None,
        help="Y 方向平移量（默认 config.RETRACK_NUDGE_M）",
    )
    return ap.parse_args()


def main() -> None:
    args = parse_args()

    from utils.robot import load_robot
    from utils.gripper import init_gripper
    from utils.ik import init_ik

    from config import COLOR_RGBA, CUBE_HALF, HOME_POS, RETRACK_NUDGE_M
    from grasp.adaptive_bridge import grasp_and_lift, import_b2_afc
    from vision.camera import CameraConfig

    afc = import_b2_afc()
    afc.connect_pybullet(direct=bool(args.direct))
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setPhysicsEngineParameter(numSolverIterations=120, numSubSteps=4)

    robot_data = load_robot()
    init_gripper(robot_data)
    init_ik(robot_data)
    robot = robot_data["robot"]
    arm_joints = robot_data["arm_joints"]
    ee_link = robot_data["ee_link"]

    # 固定初始位（可达）
    xy0 = (0.38, -0.08)
    pos0 = [xy0[0], xy0[1], CUBE_HALF]
    body = afc.load_iron_block(pos0, mass=0.55, pin=True)
    p.changeVisualShape(body, -1, rgbaColor=COLOR_RGBA["red"])
    afc.settle(15)
    afc.pin_body(body, pos0, afc.TABLE_ORN)

    home_pos = np.array(HOME_POS, dtype=float)
    home_orn = p.getQuaternionFromEuler(afc.HOME_EULER)
    afc.open_gripper(robot)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, home_pos, home_orn, afc.OPEN, steps=160,
        )
    except RuntimeError as e:
        print(f"  [HOME] {e}")
    off_local = afc.calibrate_off_local(robot, ee_link)

    dy = float(args.nudge_dy) if args.nudge_dy is not None else float(RETRACK_NUDGE_M)

    def nudge_fn(bid: int) -> None:
        pos, orn = p.getBasePositionAndOrientation(bid)
        new = [float(pos[0]), float(pos[1]) + dy, float(CUBE_HALF)]
        print(f"  [NUDGE] {np.round(pos[:2], 3)} → {np.round(new[:2], 3)}  dy={dy:.3f}m")
        afc.pin_body(bid, new, orn)

    print("\n========== M6 重跟踪 Demo ==========")
    print(f"  retrack={args.retrack}  nudge={args.nudge}  dy={dy:.3f}m")
    print(f"  初始 GT xy={xy0}")
    print("====================================\n")

    # 初始抓取目标 = 初始位（扰动前）；retrack 负责追到新位
    grasp_pos = np.array([xy0[0], xy0[1], CUBE_HALF], dtype=float)
    result = grasp_and_lift(
        afc, robot, arm_joints, ee_link, body, grasp_pos, off_local,
        "IronBlock", home_pos,
        soft_object=False, mass=0.55,
        save_force_log=not bool(args.no_force_log),
        enable_retrack=bool(args.retrack),
        retrack_color="red",
        camera_cfg=CameraConfig(),
        nudge_fn=nudge_fn if args.nudge else None,
    )

    # 成功后松开放回（本 demo 不跑分拣）
    if result.get("held"):
        cur = afc.get_current_tcp(robot, ee_link)
        for w in np.linspace(result["width"], afc.OPEN, 50):
            afc.set_gripper(robot, float(w), force=300, keep_center=True)
            p.stepSimulation()
            time.sleep(afc.SIM_DT)
        afc.open_gripper(robot)
        pos = p.getBasePositionAndOrientation(body)[0]
        afc.pin_body(body, [pos[0], pos[1], CUBE_HALF], afc.TABLE_ORN)

    final = p.getBasePositionAndOrientation(body)[0]
    print("\n========== M6 汇总 ==========")
    print(f"  grasp_ok={result.get('success')}  held={result.get('held')}")
    print(f"  retrack_n={result.get('retrack_n', 0)}")
    print(f"  final_xy=({final[0]:.3f},{final[1]:.3f})")
    ok = bool(result.get("success"))
    if args.retrack and args.nudge:
        ok = ok and int(result.get("retrack_n", 0)) >= 1
        if int(result.get("retrack_n", 0)) < 1:
            print("  FAIL: 期望至少 1 次 [RETRACK] 触发")
    print(f"总体: {'PASS' if ok else 'FAIL'}")
    print("============================\n")

    if not args.direct:
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
