"""
utils/scene.py
==============
仿真场景加载工具（入口：load_scene）。

功能：
  - 加载地面 plane
  - 放置「重物 heavy」与「易碎物 fragile」两个 cube_small
  - 按定义设置质量、摩擦与外观颜色

说明：主实验脚本 1/2/3 多数自建场景；本模块供快速原型或旧接口复用。
"""
import pybullet as p
import pybullet_data


def load_scene():
    """
    加载默认场景：地面 + 重物 + 易碎物
    返回: (plane_id, [(name, body_id), ...])
    """
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # cube_small.urdf 边长约 0.05m，中心高度取 0.025 使底面贴地
    object_defs = [
        {
            "name": "heavy",
            "urdf": "cube_small.urdf",
            "position": [0.30, -0.10, 0.025],
            "mass": 0.50,
            "lateralFriction": 1.5,
            "spinningFriction": 0.1,   # 【修改说明】原 spinning/rolling=1 过大，易导致物体“粘滞”
            "rollingFriction": 0.01,
            "scaling": 1.0,
            "color": [0.8, 0.1, 0.1, 1]
        },
        {
            "name": "fragile",
            "urdf": "cube_small.urdf",
            "position": [0.30, 0.10, 0.025],
            "mass": 0.02,
            "lateralFriction": 0.5,
            "spinningFriction": 0.1,
            "rollingFriction": 0.1,
            "scaling": 1.0,
            "color": [0.1, 0.2, 0.8, 1]
        }
    ]

    plane = p.loadURDF("plane.urdf")
    loaded_objects = []

    for obj in object_defs:
        body = p.loadURDF(
            obj["urdf"],
            basePosition=obj["position"],
            globalScaling=obj.get("scaling", 1.0)
        )
        p.changeDynamics(
            body, -1,
            mass=obj["mass"],
            lateralFriction=obj["lateralFriction"],
            spinningFriction=obj["spinningFriction"],
            rollingFriction=obj["rollingFriction"]
        )
        p.changeVisualShape(body, -1, rgbaColor=obj["color"])
        loaded_objects.append((obj["name"], body))

    print("场景加载完成:", loaded_objects)
    return plane, loaded_objects