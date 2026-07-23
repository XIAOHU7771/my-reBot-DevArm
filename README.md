# my-reBot-DevArm
基于 PyBullet 的 reBot-DevArm 六轴机械臂虚拟仿真完整实现
仓库地址：https://github.com/XIAOHU7771/my-reBot-DevArm

[![Python 3.10+](https://img.shields.io/badge/Python-%3E%3D3.10-blue)](https://www.python.org/)
[![Platform](https://img.shields.io/badge/Platform-Windows%20%7C%20Linux%20%7C%20macOS-lightgrey)]()
[![Release Demo](https://img.shields.io/github/v/release/XIAOHU7771/my-reBot-DevArm)](https://github.com/XIAOHU7771/my-reBot-DevArm/releases)

## 一、项目背景与任务说明
本项目为机械臂仿真课程两项综合任务完整实现：
1. **任务B：机械臂运动仿真与虚拟控制**
在无实体硬件条件下，基于PyBullet搭建reBot-DevArm六轴机械臂虚拟环境，实现笛卡尔坐标输入、逆运动学求解、末端平滑点位运动，输出运动仿真演示视频。

2. **任务B2：基于物理仿真环境的自适应力控抓取（进阶任务）**
在夹爪集成虚拟力传感器，实现力反馈闭环控制；区分刚性铁块、柔性海绵两种物体安全抓取；增加滑移检测与动态加紧补偿算法，对比有无力控抓取效果。

> 核心工程难点：URDF左右夹爪关节行程不一致，常规比例开合会造成夹持中心漂移、单侧受力过大。本项目设计**指心恒定对称开合算法**，保证两指几何中心不变，消除受力不均问题。

## 二、两大独立工程功能说明
仓库分为两套完全独立仿真工程，分别对应基础运动任务、力控抓取进阶任务：
### 1. simulation（任务B：基础点位运动仿真）
对应基础机械臂运动仿真需求，实现无抓取的纯点位控制：
- 加载reBot-DevArm完整URDF机械臂三维模型
- 终端交互式输入末端TCP三维坐标`(x,y,z)`
- 调用PyBullet数值逆运动学IK求解关节角度
- 线性插值生成平滑轨迹，避免机械臂抖动
- 内置3D GUI可视化仿真界面
- 配套工具脚本快速查询机械臂关节/Link索引

### 2. simulation2（任务B2：自适应力控抓取进阶仿真）
对应力传感、力控、滑移补偿完整进阶需求：
- 夹爪对称开合控制：指心不变等量回缩策略
- 双路力采集：接触点正压力、关节力矩传感器数据读取
- 实时打印力数据、自动导出力时序CSV文件、绘制力变化曲线
- 自适应闭环力控逻辑：达到安全夹持力自动停止夹紧
- 双物体场景：刚性铁块、柔性海绵抓取对比实验
- 滑移对冲算法：检测下滑外力自动增大夹持力防止掉落
- 支持开启/关闭力控模式，直观对比捏碎/稳定抓取效果

## 三、仓库完整目录结构
my-reBot-DevArm/
├── simulation/                 # 任务 B 基础点位运动仿真工程
│   ├── find_ee.py              # 工具：查询机械臂关节、Link 索引
│   └── rebot_sim.py            # 主程序：笛卡尔坐标逆运动学控制
├── simulation2/                # 任务 B2 力控抓取进阶仿真工程
│   ├── README.md               # 力控模块详细说明
│   ├── requirements.txt        # 力控模块专属依赖
│   ├── 1.Force_Sensor_Simulation.py      # 虚拟力传感器采集演示
│   ├── 3.adaptive_force_control_grasp.py # 自适应力控抓取主程序
│   ├── 4.slip_compensation_test.py       # 滑移补偿测试脚本
│   ├── utils/
│   │   └── gripper.py          # 夹爪对称开合、力均衡控制工具库
│   └── force_data/             # 自动输出：力曲线图片、CSV 数据
├── urdf/                       # 共用机械臂 URDF 三维模型与 STL 网格
├── requirements.txt            # 项目全局基础依赖
├── .gitignore
└── README.mdplaintext
## 四、环境配置
### 1. 环境要求
- Python >= 3.10
- 操作系统：Windows / Linux / macOS
- 核心依赖：pybullet、numpy、matplotlib

### 2. 一键部署流程
#### ① 克隆仓库
```bash
git clone https://github.com/XIAOHU7771/my-reBot-DevArm.git
cd my-reBot-DevArm
② 创建并激活虚拟环境Windows PowerShellpowershellpython -m venv .venv
.\.venv\Scripts\Activate.ps1
Linux / macOSbashpython3 -m venv .venv
source .venv/bin/activate
③ 安装依赖bash# 全局基础依赖
pip install pybullet numpy matplotlib
# simulation2力控模块额外依赖
pip install -r simulation2/requirements.txt

Windows 安装 PyBullet 报错解决方案：安装 Microsoft C++ Build Tools，勾选「使用 C++ 的桌面开发」工作负载。
五、运行使用教程5.1 simulation 基础点位运动（任务 B）
首次运行查询末端 Link 索引
bashpython simulation/find_ee.py
输出所有关节与 Link 编号，记录end_link索引，用于主程序配置。
启动笛卡尔坐标控制仿真
bashpython simulation/rebot_sim.py
终端提示输入目标坐标，格式示例：plaintext0.15, 0.0, 0.15
推荐安全工作区间：x:0.05~0.25, y:-0.15~0.15, z:0.05~0.25
任务产出：运行录屏，展示机械臂自动求解 IK 并平滑运动至指定坐标。
5.2 simulation2 自适应力控抓取（任务 B2）进入力控仿真目录：bashcd simulation2

虚拟力传感器单独测试，记录力曲线
bashpython 1.Force_Sensor_Simulation.py

自适应力控抓取（推荐，安全抓取铁块 / 海绵）
bashpython 3.adaptive_force_control_grasp.py --force-control

无力控对照实验（无反馈，硬挤压易损坏柔性物体）
bashpython 3.adaptive_force_control_grasp.py --no-force-control --only sponge

滑移补偿抗掉落测试
bashpython 4.slip_compensation_test.py --compensate

任务产出：对比录屏 Demo，分别展示无力控挤压变形、启用力控稳定抓取两种效果。
六、核心算法原理6.1 逆运动学 IK 求解流程
输入：末端目标三维坐标 (x,y,z) + 当前各关节角度
迭代求解（最大 500 次迭代）

计算当前末端位置与目标的误差
求解雅可比矩阵更新关节角度
更新公式：\(\theta_{new} = \theta_{old} + J^+ \cdot \Delta x\)


收敛判定：位置误差小于阈值 1e-5 时停止迭代，输出 6 轴关节角度
6.2 夹爪指心恒定对称开合算法URDF 左关节行程 0~0.05m，右关节 0~0.0715m，直接同比例开合会造成夹持中心偏移。
采用等量回缩策略：
\(\begin{cases}
d = RIGHT\_UPPER - width \\
q_{left} = LEFT\_UPPER - d \\
q_{right} = RIGHT\_UPPER - d
\end{cases}\)
保证 q_right - q_left 差值恒定，夹持几何中心不漂移，左右指接触压力均衡。6.3 自适应力均衡闭环控制
实时读取左右夹爪接触正压力；
压力差值修正关节位置，压力大的一侧轻微松开，平衡双侧受力；
若平均夹持力小于目标安全力，同步向内小幅收紧；
滑移检测：外力下滑导致摩擦力下降时，自动提升夹持力防止滑落。
七、任务交付产出说明任务 B 交付内容
完整开源代码托管于本 GitHub 仓库；
基础点位运动仿真演示视频（上传至 Release）；
规范 README 文档、可直接运行的仿真脚本。
任务 B2 进阶交付内容
新增 simulation2 全套力控抓取代码、夹爪工具库；
力控对比演示视频（Release 附带）：

关闭力控：夹爪持续闭合挤压，海绵过度形变；
启用力控：接触物体达到设定力自动停止，稳定抓取重物与柔性物体；


滑移补偿防掉落实验录像；
力数据自动导出 CSV 与可视化力曲线。
八、故障排查

PyBullet 安装编译报错
安装 Microsoft C++ Build Tools，勾选桌面 C++ 开发组件，重启终端后重新安装。


IK 求解失败、机械臂不动


重新运行find_ee.py确认末端 link 索引配置正确；
目标坐标超出工作空间，更换推荐区间坐标；
增大 IK 最大迭代次数maxNumIterations至 1000。

夹爪左右受力差距过大
确认开合模式keep_center=True开启指心对称策略，关闭传统比例映射。
九、许可证本项目基于 reBot-DevArm 开源机械臂硬件项目，遵循 CERN-OHL-W-2.0 开源协议。十、致谢
reBot-DevArm 开源六轴机械臂硬件项目
PyBullet 多刚体物理仿真引擎
机械臂逆运动学、力控抓取相关开源参考工程