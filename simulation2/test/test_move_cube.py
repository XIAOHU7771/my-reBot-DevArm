import pybullet as p
import time
import sys
from pathlib import Path
# 获取simulation2文件夹路径，加入Python搜索路径
root_path = Path(__file__).parent.parent
sys.path.append(str(root_path))

from utils.robot import load_robot
from utils.scene import load_scene
from utils.ik import move_to



def main():

    p.connect(p.GUI)

    p.setGravity(
        0,
        0,
        -9.8
    )


    robot_info = load_robot()


    robot = robot_info["robot"]


    plane,cube = load_scene(robot)



    # cube上方目标点

    target = [
        0.30,
        0,
        0.12
    ]


    print(
        "移动目标:",
        target
    )


    move_to(

        robot,

        robot_info["arm_joints"],

        robot_info["ee_link"],

        target,

        steps=240

    )


    while True:

        p.stepSimulation()

        time.sleep(
            1/240
        )



if __name__=="__main__":

    main()