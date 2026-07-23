import pybullet as p
import time
import sys
from pathlib import Path
# 获取simulation2文件夹路径，加入Python搜索路径
root_path = Path(__file__).parent.parent
sys.path.append(str(root_path))


from utils.ik import move_tcp
from utils.robot import load_robot
from utils.scene import load_scene




p.connect(p.GUI)


p.setGravity(
    0,
    0,
    -9.8
)


robot_info = load_robot()


robot = robot_info["robot"]


load_scene(robot)



target_tcp = [
    0.30,
    0,
    0.10
]


print(
    "移动TCP目标:",
    target_tcp
)



move_tcp(
        robot,
        joints,
        ee,
        [
        0.20,
        0,
        0.22
        ]
        )


while True:

    p.stepSimulation()

    time.sleep(
        1/240
    )

