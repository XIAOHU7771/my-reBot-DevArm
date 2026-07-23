# reBot-DevArm 力觉 / 力控仿真（simulation2）

基于 PyBullet 的 reBot-DevArm 机械臂力传感器、自适应力控抓取与滑移对冲仿真。

## 📌 项目简介

本目录在基础 TCP 点动仿真之上，实现夹爪虚拟力/触觉传感、按接触刚度自适应选力的闭环抓取，以及抬升后对外力扰动的滑移对冲对照实验。机械臂模型使用仓库根目录下的 `urdf/00-arm-rs_asm-v3`。

### 核心功能

- ✅ 加载 reBot-DevArm 机械臂 URDF（含平行夹爪）
- ✅ 虚拟力/触觉：接触点正压力 + 关节反作用力读数
- ✅ 侧夹闭合过程实时打印力，并导出 CSV / 曲线
- ✅ 自适应力控：探刚度 → 选安全力 F_safe → PID 恒力抓取
- ✅ 无力控对照：大力闭合，软物可被“捏扁”可视化
- ✅ 滑移对冲：抬升后施加向下外力，检测滑移并自动加紧
- ✅ 3D 可视化 GUI（也支持 `--direct` 无界面批跑）

## 📁 项目结构

```text
simulation2/
├── 1.Force_Sensor_Simulation.py       # 任务1：虚拟力/触觉传感
├── 2.adaptive_force_control_grasp.py  # 任务2：自适应力控抓取
├── 3.slip_compensation_test.py        # 任务3：滑移对冲对照
├── requirements.txt                   # 本目录依赖
├── README.md                          # 本说明
├── force_data/                        # 运行后生成的 CSV / 曲线图
└── utils/                             # 公共工具
    ├── robot.py                       # 加载 URDF，解析臂/夹爪索引
    ├── gripper.py                     # 夹爪开合与力读数
    ├── ik.py                          # TCP 逆解与数值精调
    ├── scene.py                       # 可选默认场景
    └── __init__.py
```

仓库根目录中与本目录相关的路径：

```text
reBot-DevArm/
├── simulation/          # 基础 TCP 点动（见 simulation/README.md）
├── simulation2/         # 本目录
├── urdf/                # 机械臂 URDF（00-arm-rs_asm-v3）
├── requirements.txt
└── README.md
```

## 🛠 环境要求

| 项目 | 要求 |
|------|------|
| Python | 3.10+ |
| 操作系统 | Windows / Linux / macOS |
| 依赖库 | PyBullet, NumPy, Matplotlib |

## 📦 安装步骤

### 1. 克隆项目

```bash
git clone https://github.com/XIAOHU7771/my-reBot-DevArm.git
cd my-reBot-DevArm
```

### 2. 创建并激活虚拟环境

**Windows:**

```bash
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

**Linux / macOS:**

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. 安装依赖

```bash
pip install -r simulation2/requirements.txt
# 或在仓库根目录：
pip install -r requirements.txt
```

注意：在 Windows 上安装 PyBullet 时，如果提示缺少 Visual C++ 编译工具，请先安装 [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)，确保勾选“使用 C++ 的桌面开发”工作负载。

## 🚀 运行方法

建议先进入本目录（CSV / 图片默认写到 `force_data/`，依赖当前工作目录）：

```bash
cd simulation2
```

### 任务 1：虚拟力传感器

在左右指内侧读取接触正压力，侧夹闭合时实时打印并记录时序。

```bash
python 1.Force_Sensor_Simulation.py
python 1.Force_Sensor_Simulation.py --direct --no-show
python 1.Force_Sensor_Simulation.py --cube 0.4 0.2 0.04 --mass 0.15 --friction 2.5 --target-force 12
```

GUI 模式下会并排打开 PyBullet 仿真窗与力曲线实时窗。

### 任务 2：自适应力控抓取

同场景放置铁块（重、硬）与海绵（轻、易压）。力控开启时按刚度选安全力并 PID 维持；关闭时大力闭合，易碎物可能被捏扁。

```bash
# 力控开启（默认也可改脚本内 ENABLE_FORCE_CONTROL）
python 2.adaptive_force_control_grasp.py --force-control

# 力控关闭对照（建议只看海绵）
python 2.adaptive_force_control_grasp.py --no-force-control --only sponge

# 无 GUI 批跑
python 2.adaptive_force_control_grasp.py --force-control --direct --no-show
```

### 任务 3：滑移对冲

先力控抓起，再对物体施加持续向下外力。对冲开启时检测滑移并略微加紧；关闭时固定握宽，易掉落。

```bash
python 3.slip_compensation_test.py --compensate
python 3.slip_compensation_test.py --no-compensate
python 3.slip_compensation_test.py --direct --no-show --disturb 40
```

本脚本会动态加载 `2.adaptive_force_control_grasp.py` 中的抓取与读力工具。

### 常用参数一览

| 脚本 | 常用参数 | 说明 |
|------|----------|------|
| 1 | `--direct` / `--no-show` | 无 GUI / 不弹最终图 |
| 1 | `--target-force` | 目标总正压力 (N) |
| 2 | `--force-control` / `--no-force-control` | 力控开/关 |
| 2 | `--only iron\|sponge\|all` | 只测某一类物体 |
| 3 | `--compensate` / `--no-compensate` | 滑移对冲开/关 |
| 3 | `--disturb` | 向下外力大小 (N) |

## 📊 代码架构

```text
┌─────────────────────────────────────────────────────────────┐
│           实验脚本 1 / 2 / 3（场景 + 控制逻辑）               │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                      utils/ 公共层                          │
├─────────────────────────────────────────────────────────────┤
│  robot.py   加载 URDF，解析 joint1~6 / 夹爪 / gripper_end   │
│  gripper.py 开口映射（指心保持）、接触力 / 关节反力          │
│  ik.py      DLS 初值 + 雅可比精调，TCP 位姿控制             │
│  scene.py   可选默认桌面场景（重物 / 易碎物）                │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                    PyBullet 物理引擎                        │
├─────────────────────────────────────────────────────────────┤
│  • URDF 解析与刚体接触                                      │
│  • getContactPoints（正压力 / 切向摩擦）                    │
│  • enableJointForceTorqueSensor（关节反力）                 │
│  • 逆运动学 + 位置电机跟踪                                  │
│  • 3D 可视化渲染                                            │
└─────────────────────────────────────────────────────────────┘
```

### 任务流程（简图）

**任务 1 — 力传感**

```text
加载机器人 → 标定指心偏移 → 接近立方体 → 闭合
    → 读接触正压力 / 关节反力 → 记录 CSV + 曲线
```

**任务 2 — 自适应力控**

```text
放置铁块 + 海绵（冻结贴桌）→ 夹爪去找物体
  力控 ON : 接触 → 探刚度 k≈ΔF/Δw → 选 F_safe → PID 恒力 → 抬升
  力控 OFF: 大力闭合 → 软物捏扁可视化 / 可能失败
```

**任务 3 — 滑移对冲**

```text
力控抓起 → 临界握持 → 施加向下外力
  对冲 ON : 检测相对滑移 / 力异常 → 对称加紧 → 稳住
  对冲 OFF: 固定握宽 → 易滑落
```

### 数据输出

运行结果默认写入 `force_data/`：

- `gripper_force_*.csv` / `.png` — 任务 1
- `adaptive_fc_*` / `blind_*` — 任务 2（力控 / 无力控）
- `slip_comp_ON_*` / `slip_comp_OFF_*` — 任务 3

## 📝 许可证

本项目基于 reBot-DevArm 开源项目，遵循 CERN-OHL-W-2.0 许可证。

## 🤝 致谢

- [reBot-DevArm](https://github.com/) — 开源硬件项目
- PyBullet — 物理仿真引擎
- 基础 TCP 点动见同仓库 [`simulation/`](../simulation/)
