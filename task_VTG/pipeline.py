"""
task_VTG/pipeline.py
====================
M3/M4：流水线状态机（config 固定坐标 或 视觉目标）。
"""

from __future__ import annotations

import traceback
from enum import Enum
from typing import Any

import numpy as np
import pybullet as p

from config import HOME_POS, OBJECTS, object_pos3
from grasp.adaptive_bridge import grasp_and_lift, place_held_object
from motion.retreat import go_home
from sort.place_zones import in_zone, zone_name


class PipelineState(Enum):
    IDLE = "IDLE"
    LOAD_TARGETS = "LOAD_TARGETS"
    DETECT = "DETECT"
    SELECT_NEXT = "SELECT_NEXT"
    APPROACH_PRE = "APPROACH_PRE"
    FORCE_GRASP = "FORCE_GRASP"
    LIFT = "LIFT"
    TRANSPORT = "TRANSPORT"
    PLACE = "PLACE"
    RETREAT = "RETREAT"
    DONE = "DONE"
    SKIP = "SKIP"


def _log_state(state: PipelineState, msg: str = "") -> None:
    extra = f"  {msg}" if msg else ""
    print(f"[STATE] {state.value}{extra}")


def run_pipeline(
    afc: Any,
    robot_data: dict,
    bodies: dict[str, int],
    objects: list[dict] | None = None,
    *,
    save_force_log: bool = True,
    source: str = "config",
    camera_cfg: Any = None,
    bodies_by_color: dict[str, int] | None = None,
    annotate_path: str | None = None,
    annotate_title: str | None = None,
) -> dict:
    """
    依次：抓取+抬升 → 分拣放置 → 回安全位。

    source:
      - "config"：用传入 objects / OBJECTS 的固定 xy
      - "vision"：回 HOME 后检测，用视觉 xy（Z 仍 CUBE_HALF）
    """
    robot = robot_data["robot"]
    arm_joints = robot_data["arm_joints"]
    ee_link = robot_data["ee_link"]
    home_pos = np.array(HOME_POS, dtype=float)
    home_orn = p.getQuaternionFromEuler(afc.HOME_EULER)

    _log_state(PipelineState.IDLE)

    afc.open_gripper(robot)
    afc.settle(10)
    try:
        afc.move_tcp(
            robot, arm_joints, ee_link, home_pos, home_orn, afc.OPEN, steps=180,
        )
    except RuntimeError as e:
        print(f"  [HOME] {e}")
    off_local = afc.calibrate_off_local(robot, ee_link)

    detect_info: list[dict] = []
    annotate_saved: str | None = None
    if source == "vision":
        _log_state(PipelineState.DETECT, "capture + color localize")
        from vision.camera import CameraConfig
        from vision.targets import (
            compare_to_gt,
            detect_targets_with_frame,
            targets_to_objects,
        )

        cfg = camera_cfg if camera_cfg is not None else CameraConfig()
        frame, targets = detect_targets_with_frame(cfg, bodies_by_color=bodies_by_color)
        if not targets:
            _log_state(PipelineState.SKIP, "reason=detect_miss 未检出任何目标")
            _log_state(PipelineState.DONE)
            return {
                "results": [],
                "all_ok": False,
                "source": source,
                "detect_info": [],
                "reason": "detect_miss",
                "annotate_path": None,
            }
        objects = targets_to_objects(targets)
        if annotate_path:
            from vision.annotate import annotate_from_targets, save_annotation

            annot = annotate_from_targets(
                frame.rgb, targets, title=annotate_title or "M5 vision annotate",
            )
            annotate_saved = save_annotation(annotate_path, annot)
            print(f"  [ANNOTATE] {annotate_saved}")
        # 与 GT 对比（若有 body）
        gt_xy = {}
        if bodies_by_color:
            for color, bid in bodies_by_color.items():
                pos = p.getBasePositionAndOrientation(bid)[0]
                gt_xy[color] = (float(pos[0]), float(pos[1]))
            detect_info = compare_to_gt(targets, gt_xy)
            print("  --- 视觉 vs GT ---")
            for row in detect_info:
                print(
                    f"  {row['color']}: vis={np.round(row['xy_vis'], 4)} "
                    f"gt={np.round(row['xy_gt'], 4)} "
                    f"err_xy={row['err_xy_cm']:.2f}cm"
                )
        missing = set(COLOR_META_COLORS()) - {t.color for t in targets}
        if missing:
            print(f"  [DETECT] 缺少颜色: {sorted(missing)}（将只处理已检出）")
    else:
        if objects is None:
            objects = OBJECTS

    _log_state(PipelineState.LOAD_TARGETS, f"source={source} n={len(objects)}")

    results: list[dict] = []
    id_list = [o["id"] for o in objects]

    for idx, obj in enumerate(objects):
        _log_state(PipelineState.SELECT_NEXT, f"{obj['id']} ({idx+1}/{len(objects)})")
        if obj["id"] not in bodies:
            _log_state(PipelineState.SKIP, f"{obj['id']} reason=no_body")
            results.append(_result_row(obj, reason="no_body"))
            continue

        body = bodies[obj["id"]]
        kind = obj["kind"]
        label = obj["label"]
        soft = bool(obj["soft"])
        mass = float(obj["mass"])
        # 抓取目标：视觉/配置 xy；物体钉在真实当前位置（不把物体挪到估计点）
        grasp_pos = np.array(object_pos3(obj), dtype=float)
        real_pos = np.array(p.getBasePositionAndOrientation(body)[0], dtype=float)
        real_pos[2] = grasp_pos[2]

        for oid in id_list:
            if oid == obj["id"] or oid not in bodies:
                continue
            other = bodies[oid]
            opos = p.getBasePositionAndOrientation(other)[0]
            afc.freeze_body(other, opos, afc.TABLE_ORN)
            afc.set_robot_obj_collision(robot, other, enable=False)

        afc.pin_body(body, real_pos, afc.TABLE_ORN)
        afc.settle(5)

        _log_state(PipelineState.APPROACH_PRE, f"target_xy={grasp_pos[:2]}")
        _log_state(PipelineState.FORCE_GRASP)
        _log_state(PipelineState.LIFT)
        try:
            g = grasp_and_lift(
                afc, robot, arm_joints, ee_link, body, grasp_pos, off_local,
                label, home_pos,
                soft_object=soft, mass=mass, save_force_log=save_force_log,
            )
        except Exception as e:
            print(f"  [异常] {label}: {e}")
            traceback.print_exc()
            g = {
                "label": label, "success": False, "held": False,
                "kind": "未知", "lifted": 0.0, "crushed": False,
                "final_force": 0.0, "f_target": 0.0,
            }

        row = _result_row(obj)
        row["grasp_ok"] = bool(g.get("success"))

        if not g.get("held"):
            reason = "grasp_fail"
            _log_state(PipelineState.SKIP, f"{obj['id']} reason={reason}")
            row["reason"] = reason
            row["success"] = False
            results.append(row)
            go_home(afc, robot, arm_joints, ee_link, home_pos, home_orn)
            _restore_others(afc, robot, bodies, objects, obj["id"])
            continue

        _log_state(PipelineState.TRANSPORT)
        _log_state(PipelineState.PLACE)
        try:
            pr = place_held_object(
                afc, robot, arm_joints, ee_link, body, kind, g["orn"], g["width"],
            )
        except Exception as e:
            print(f"  [PLACE 异常] {e}")
            traceback.print_exc()
            pr = {"placed": False, "final_pos": None}

        final = pr.get("final_pos")
        if final is None:
            final = list(p.getBasePositionAndOrientation(body)[0])
        row["final_pos"] = final
        row["placed_ok"] = bool(pr.get("placed"))
        row["in_zone"] = in_zone(final, kind)
        if not row["placed_ok"]:
            row["reason"] = "place_fail"
        elif not row["in_zone"]:
            row["reason"] = "place_miss"
        else:
            row["reason"] = ""
        row["success"] = row["grasp_ok"] and row["placed_ok"] and row["in_zone"]
        if not row["success"]:
            _log_state(PipelineState.SKIP, f"{obj['id']} reason={row['reason']}")
        results.append(row)

        _log_state(PipelineState.RETREAT)
        go_home(afc, robot, arm_joints, ee_link, home_pos, home_orn)
        _restore_others(afc, robot, bodies, objects, obj["id"])
        afc.settle(15)

    _log_state(PipelineState.DONE)
    expected = 2 if source == "vision" else len(objects)
    # 视觉源期望红+黄都成功；若只检出一个则 all_ok=False
    all_ok = (
        len(results) >= expected
        and all(r.get("success") for r in results)
        and (source != "vision" or len(results) == expected)
    )
    return {
        "results": results,
        "all_ok": all_ok,
        "source": source,
        "detect_info": detect_info,
        "annotate_path": annotate_saved if source == "vision" else None,
    }


def COLOR_META_COLORS() -> set[str]:
    from config import COLOR_META

    return set(COLOR_META.keys())


def _result_row(obj: dict, reason: str = "") -> dict:
    kind = obj["kind"]
    return {
        "id": obj["id"],
        "label": obj.get("label", obj["id"]),
        "kind": kind,
        "grasp_ok": False,
        "placed_ok": False,
        "zone": zone_name(kind),
        "final_pos": None,
        "in_zone": False,
        "success": False,
        "reason": reason,
    }


def _restore_others(afc, robot, bodies, objects, current_id: str) -> None:
    """恢复非当前物体的碰撞与冻结展示。"""
    for obj in objects:
        if obj["id"] == current_id:
            continue
        if obj["id"] not in bodies:
            continue
        body = bodies[obj["id"]]
        pos = p.getBasePositionAndOrientation(body)[0]
        afc.freeze_body(body, pos, afc.TABLE_ORN)
        afc.set_robot_obj_collision(robot, body, enable=True)
