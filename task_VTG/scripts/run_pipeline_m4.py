"""
task_VTG/scripts/run_pipeline_m4.py
===================================
M4 入口：随机摆放 + 视觉定位 + M3 力控分拣（不手改坐标）。

运行（仓库根目录）:
  python task_VTG/scripts/run_pipeline_m4.py --direct --no-force-log
  python task_VTG/scripts/run_pipeline_m4.py --seed 7 --direct
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

if "--direct" in sys.argv or "--no-show" in sys.argv:
    import matplotlib

    matplotlib.use("Agg")


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="M4/M5: 视觉接入抓取+分拣")
    ap.add_argument("--direct", action="store_true", help="无 GUI")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--no-force-log", action="store_true")
    ap.add_argument("--seed", type=int, default=None, help="随机种子（可复现）")
    ap.add_argument(
        "--annotate",
        action="store_true",
        help="M5：DETECT 后保存标注图到 debug_images/",
    )
    return ap.parse_args()


def _away_from_zones(xy: np.ndarray, clear: float) -> bool:
    from config import ZONE_A_SOFT, ZONE_B_HARD

    for z in (ZONE_A_SOFT, ZONE_B_HARD):
        if float(np.linalg.norm(xy - np.asarray(z, dtype=float))) < clear:
            return False
    return True


def sample_spawn_xy(rng: np.random.Generator, existing: list[np.ndarray]) -> np.ndarray:
    """在工作台采样一个与已有点、放置区都保持距离的 XY。"""
    from config import (
        SPAWN_MIN_SEP,
        SPAWN_X_RANGE,
        SPAWN_Y_RANGE,
        SPAWN_ZONE_CLEAR,
    )

    for _ in range(200):
        xy = np.array(
            [
                rng.uniform(*SPAWN_X_RANGE),
                rng.uniform(*SPAWN_Y_RANGE),
            ],
            dtype=float,
        )
        if not _away_from_zones(xy, SPAWN_ZONE_CLEAR):
            continue
        if any(float(np.linalg.norm(xy - e)) < SPAWN_MIN_SEP for e in existing):
            continue
        return xy
    raise RuntimeError("随机摆放采样失败：请放宽 SPAWN_* 参数")


def paint_for_vision(body: int, color: str) -> None:
    from config import COLOR_RGBA

    p.changeVisualShape(body, -1, rgbaColor=COLOR_RGBA[color])


def main() -> None:
    args = parse_args()
    seed = int(args.seed) if args.seed is not None else int(time.time()) % 100000
    rng = np.random.default_rng(seed)

    from utils.robot import load_robot
    from utils.gripper import init_gripper
    from utils.ik import init_ik

    from config import COLOR_META, CUBE_HALF
    from grasp.adaptive_bridge import import_b2_afc
    from pipeline import run_pipeline
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

    # 随机 GT 位姿：红硬 / 黄软
    xy_red = sample_spawn_xy(rng, [])
    xy_yel = sample_spawn_xy(rng, [xy_red])
    gt = {
        "red": (float(xy_red[0]), float(xy_red[1])),
        "yellow": (float(xy_yel[0]), float(xy_yel[1])),
    }

    print("\n========== M4 视觉流水线 ==========")
    print(f"  seed={seed}")
    print(f"  GT red/hard   xy={gt['red']}")
    print(f"  GT yellow/soft xy={gt['yellow']}")
    print("  目标来源: vision（运行中不手改坐标）")
    print("====================================\n")

    bodies: dict[str, int] = {}
    bodies_by_color: dict[str, int] = {}

    # 铁块（红）
    meta_r = COLOR_META["red"]
    pos_r = [gt["red"][0], gt["red"][1], CUBE_HALF]
    iron = afc.load_iron_block(pos_r, mass=float(meta_r["mass"]), pin=True)
    paint_for_vision(iron, "red")
    bodies[meta_r["id"]] = iron
    bodies_by_color["red"] = iron

    # 海绵（黄）
    meta_y = COLOR_META["yellow"]
    pos_y = [gt["yellow"][0], gt["yellow"][1], CUBE_HALF]
    sponge = afc.load_sponge(pos_y, mass=float(meta_y["mass"]), pin=True)
    paint_for_vision(sponge, "yellow")
    bodies[meta_y["id"]] = sponge
    bodies_by_color["yellow"] = sponge

    p.setCollisionFilterPair(iron, sponge, -1, -1, 0)
    afc.settle(20)
    afc.pin_body(iron, pos_r, afc.TABLE_ORN)
    afc.pin_body(sponge, pos_y, afc.TABLE_ORN)

    summary: dict = {"results": [], "all_ok": False}
    annotate_path = None
    if args.annotate:
        out_dir = os.path.join(_VTG_ROOT, "debug_images")
        os.makedirs(out_dir, exist_ok=True)
        annotate_path = os.path.join(out_dir, f"m5_annotate_seed{seed}.png")

    try:
        summary = run_pipeline(
            afc,
            robot_data,
            bodies,
            objects=None,
            save_force_log=not bool(args.no_force_log),
            source="vision",
            camera_cfg=CameraConfig(),
            bodies_by_color=bodies_by_color,
            annotate_path=annotate_path,
            annotate_title=f"M5 annotate seed={seed}",
        )
    except Exception as e:
        print(f"\n[流水线异常] {type(e).__name__}: {e}")
        import traceback

        traceback.print_exc()
    finally:
        print("\n========== M4/M5 汇总 ==========")
        print(f"  seed={seed}  source={summary.get('source', 'vision')}")
        if summary.get("annotate_path"):
            print(f"  annotate={summary['annotate_path']}")
        for r in summary.get("results", []):
            flag = "OK" if r.get("success") else "FAIL"
            fp = r.get("final_pos")
            fp_s = (
                f"({fp[0]:.3f},{fp[1]:.3f},{fp[2]:.3f})"
                if fp is not None
                else "n/a"
            )
            reason = r.get("reason") or ""
            reason_s = f" reason={reason}" if reason else ""
            print(
                f"  [{flag}] {r.get('id')}: grasp={r.get('grasp_ok')} "
                f"place={r.get('placed_ok')} in_zone={r.get('in_zone')} "
                f"→ {r.get('zone')}  pos={fp_s}{reason_s}"
            )
        if summary.get("reason"):
            print(f"  pipeline_reason={summary['reason']}")
        ok = bool(summary.get("all_ok"))
        n_ok = sum(1 for r in summary.get("results", []) if r.get("success"))
        n = len(summary.get("results", []))
        print(f"总体: {'PASS' if ok else 'FAIL'}  ({n_ok}/{n})")
        print("==============================\n")

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

    if args.annotate and not summary.get("annotate_path"):
        print("[M5] 警告: 已请求 --annotate 但未生成标注图")
        if not summary.get("all_ok"):
            raise SystemExit(1)
        raise SystemExit(2)

    if not summary.get("all_ok"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
