# reBot-DevArm Simulation & Adaptive Force Control

<p align="center">
  <b>PyBullet-based Robot Simulation and Adaptive Grasping Research</b>
</p>


## Overview

本项目基于开源机械臂平台 **reBot-DevArm** 进行二次开发，使用 **PyBullet** 搭建机器人仿真环境，实现机械臂运动控制、逆运动学求解、物体抓取以及自适应力控抓取实验。

项目主要面向：

- Robot Simulation（机器人仿真）
- Motion Planning（运动规划）
- Inverse Kinematics（逆运动学）
- Robotic Grasping（机器人抓取）
- Adaptive Force Control（自适应力控）


本仓库包含两个独立实验任务：

| 模块 | 任务 | 内容 |
|----|----|----|
| simulation | 基础机械臂仿真任务 | URDF加载、IK控制、轨迹运动、Cube抓取 |
| simulation2 | 自适应力控抓取任务 | 力反馈、夹爪状态采集、抓取曲线分析 |


---

# Project Structure


```
my-reBot-DevArm
│
├── simulation
│   │
│   ├── urdf
│   │   └── Robot model files
│   │
│   ├── utils
│   │   ├── robot.py
│   │   ├── scene.py
│   │   ├── ik.py
│   │   └── gripper.py
│   │
│   ├── test_workspace.py
│   ├── workspace_gui.py
│   └── pick_cube.py
│
│
├── simulation2
│   │
│   ├── adaptive_force_control
│   │
│   ├── force_sensor
│   │
│   ├── grasp_controller
│   │
│   ├── curve_visualization
│   │
│   └── force_grasp.py
│
│
├── requirements.txt
│
└── README.md

```


---

# Environment


## Hardware

测试环境：

- CPU: Intel i9
- GPU: NVIDIA RTX 5060 Laptop GPU


## Software

- Windows 10/11
- Python 3.11
- PyBullet
- NumPy
- Matplotlib


安装依赖：

```bash
pip install -r requirements.txt
```


或者：


```bash
pip install pybullet numpy matplotlib
```


---

# Simulation

# Task 1: Basic Robot Simulation


## Introduction


`simulation` 是机械臂基础仿真任务。

主要实现：

- 机器人模型加载
- 关节控制
- 末端执行器运动
- 逆运动学求解
- Cube自动抓取


整体流程：

```
Robot Model

      ↓

PyBullet Simulation

      ↓

IK Solver

      ↓

End Effector Motion

      ↓

Grasp Object

```


---

# 1. Robot Model Loading


通过 URDF 文件加载机械臂模型：

主要包含：

- 六自由度机械臂
- 固定末端连接
- 两指平行夹爪


机器人结构：


```
base_link

    |

 link1

    |

 link2

    |

 ...

    |

 link6

    |

gripper_end

    |

left/right finger

```


---

# 2. Inverse Kinematics


通过 PyBullet 内置 IK 求解机械臂关节角度。


输入：

```
Target TCP Position

(x,y,z)
```


输出：

```
Joint1 ~ Joint6 Angle

```


控制机械臂末端移动到目标位置。


---

# 3. Gripper Control


夹爪采用两个 prismatic joint 控制：

```
joint_left

joint_right

```


支持：

- Open Gripper
- Close Gripper


初始状态：

```
Fully Open

width = 0.0715 m

```


---

# 4. Cube Grasping Task


抓取流程：


```
Open Gripper

        ↓

Move to Pre-Grasp Pose

        ↓

Move Down

        ↓

Close Gripper

        ↓

Lift Cube

```


目标：

- 验证机械臂运动能力
- 验证IK控制效果
- 完成自主抓取


---

# Run Simulation


进入目录：


```bash
cd simulation
```


运行：


```bash
python pick_cube.py
```


运行后：

- PyBullet GUI启动
- 机械臂自动运动
- 完成Cube抓取任务


---

# Simulation2

# Task 2: Adaptive Force Control Grasping


## Introduction


`simulation2` 是在基础仿真任务上的进一步扩展。

目标：

实现机械臂针对不同物体的：

> 自适应力控抓取


主要研究：

- 接触检测
- 抓取力估计
- 夹爪运动控制
- 力反馈调节


---

# System Architecture


```
Object

  |

Contact

  |

Force Sensor

  |

Controller

  |

Gripper Motion

```


---

# 1. Gripper Width Recording


实时记录夹爪状态：

采集数据：

```
Time

Gripper Width

Contact State

Force Value

```


生成：

```
Gripper Width Curve

```


曲线变化：

```

Width

0.0715m

 |

 |\
 | \
 |  \
 |   \
 |    \

0m ---------------- Time


```


表示：

夹爪从最大开口逐渐闭合。


---

# 2. Force Feedback


利用 PyBullet 物理接口获取接触信息：


```python
getJointState()

```


获取：

- Joint reaction force
- Contact information


用于判断：

```
Object Contact

        ↓

Force Increase

        ↓

Stop Closing

```


---

# 3. Adaptive Grasp Controller


控制策略：


```
Gripper Close

        ↓

Monitor Force

        ↓

Compare Threshold

        ↓

Adjust Position

        ↓

Stable Grasp

```


支持：

- 不同物体尺寸
- 不同质量
- 不同摩擦系数


---

# Experiment


## Object Parameters


测试不同物体：


| Object | Mass | Friction |
|-|-|-|
| Cube | 0.05kg | High |
| Object2 | Different | Different |


---

# Result Visualization


输出：

## Gripper Width Curve


展示：

- 夹爪闭合过程
- 接触时间
- 最终夹持状态


## Force Curve


展示：

- 接触力变化
- 稳定夹持阶段


---

# Future Work


## 1. Vision Based Grasping


加入视觉模块：

```
Camera

 ↓

Object Detection

 ↓

Pose Estimation

 ↓

IK Planning

 ↓

Grasp

```


计划结合：

- RGB Camera
- Depth Camera
- YOLO


---

## 2. Embodied Intelligence


进一步探索：

- Vision-Language-Action Model
- Robot Learning
- Large Language Model Control


实现：

```
Human Instruction

        ↓

LLM

        ↓

Robot Action

```


---

# Acknowledgement


This project is based on:

**reBot-DevArm Open Source Robot Platform**


Thanks to the open-source robotics community.


---

# License


For research and educational purposes.