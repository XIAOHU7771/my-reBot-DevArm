"""
task_VTG/scripts/run_pipeline_m3.py
===================================
M3 入口：固定坐标状态机 — 力控抓取 + 分拣放置（无视觉）。

运行（仓库根目录）:
  python task_VTG/scripts/run_pipeline_m3.py --direct
  python task_VTG/scripts/run_pipeline_m3.py
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

# --direct 下保存力控曲线需非交互后端
if "--direct" in sys.argv or "--no-show" in sys.argv:
    import matplotlib

    matplotlib.use("Agg")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M3: 无视觉抓取+分拣流水线")
    ap.add_argument("--direct", action="store_true", help="无 GUI")
    ap.add_argument("--no-show", action="store_true", help="同 direct 侧效果：Agg 后端")
    ap.add_argument(
        "--no-force-log",
        action="store_true",
        help="不写 force_data 曲线（加快）",
    )
    return ap.parse_args()


def main() -> None:
    args = parse_args()

    from utils.robot import load_robot
    from utils.gripper import init_gripper
    from utils.ik import init_ik

    from config import OBJECTS, object_pos3
    from grasp.adaptive_bridge import import_b2_afc
    from pipeline import run_pipeline
    from sort.place_zones import zone_name

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

    print("\n========== M3 无视觉流水线 ==========")
    print("  目标来源: config.OBJECTS（固定坐标）")
    print("  力控: B2 adaptive bridge")
    for obj in OBJECTS:
        print(
            f"  - {obj['id']}: xy={obj['xy']} kind={obj['kind']} "
            f"→ {zone_name(obj['kind'])}"
        )
    print("=====================================\n")

    bodies: dict[str, int] = {}
    for obj in OBJECTS:
        pos = object_pos3(obj)
        if obj["soft"]:
            body = afc.load_sponge(pos, mass=float(obj["mass"]), pin=True)
        else:
            body = afc.load_iron_block(pos, mass=float(obj["mass"]), pin=True)
        bodies[obj["id"]] = body

    # 两物体互不碰撞
    ids = list(bodies.values())
    if len(ids) >= 2:
        p.setCollisionFilterPair(ids[0], ids[1], -1, -1, 0)

    afc.settle(20)
    for obj in OBJECTS:
        afc.pin_body(bodies[obj["id"]], object_pos3(obj), afc.TABLE_ORN)

    summary = {"results": [], "all_ok": False}
    try:
        summary = run_pipeline(
            afc,
            robot_data,
            bodies,
            OBJECTS,
            save_force_log=not bool(args.no_force_log),
        )
    except Exception as e:
        print(f"\n[流水线异常] {type(e).__name__}: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("\n========== M3 汇总 ==========")
        for r in summary.get("results", []):
            flag = "OK" if r.get("success") else "FAIL"
            fp = r.get("final_pos")
            fp_s = (
                f"({fp[0]:.3f},{fp[1]:.3f},{fp[2]:.3f})"
                if fp is not None
                else "n/a"
            )
            print(
                f"  [{flag}] {r.get('id')}: grasp={r.get('grasp_ok')} "
                f"place={r.get('placed_ok')} in_zone={r.get('in_zone')} "
                f"→ {r.get('zone')}  pos={fp_s}"
            )
        ok = bool(summary.get("all_ok"))
        n_ok = sum(1 for r in summary.get("results", []) if r.get("success"))
        n = len(summary.get("results", []))
        print(f"总体: {'PASS' if ok else 'FAIL'}  ({n_ok}/{n})")
        print("============================\n")

        if not args.direct:
            print("关闭 GUI 窗口结束...")
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

    if not summary.get("all_ok"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
