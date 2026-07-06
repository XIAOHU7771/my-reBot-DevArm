```markdown
# reBot-DevArm 虚拟仿真控制

基于 PyBullet 的 reBot-DevArm 机械臂虚拟仿真与控制项目。

---

## 📌 项目简介

本项目实现了 reBot-DevArm 六轴机械臂在 PyBullet 物理引擎中的虚拟仿真与控制。用户可以通过终端输入目标空间坐标 `(x, y, z)`，机械臂末端将平滑移动到指定位置。

### 核心功能

- ✅ 加载 reBot-DevArm 机械臂 URDF 模型
- ✅ 支持用户交互式输入目标坐标
- ✅ 基于 PyBullet 内置 IK 求解器计算逆运动学
- ✅ 平滑插值轨迹，实现末端平滑运动
- ✅ 3D 可视化仿真界面

---

## 📁 项目结构

reBot-DevArm/
├── simulation/
│   ├── rebot_sim.py                        # 主控制脚本
│   ├── find_ee.py                          # 辅助脚本：查找末端链接索引
│   └── urdf/
│       └── reBot-DevArm_fixed_description/
│           ├── urdf/
│           │   └── reBot-DevArm_fixed.urdf   # 机械臂 URDF 模型
│           └── meshes/
│               ├── base_link.STL
│               ├── link1.STL ~ link6.STL
│               └── end_link.STL               # 3D 网格文件
├── .venv/                                   # Python 虚拟环境
├── README.md                                # 项目说明文档
└── LICENSE                                  # 开源协议
```

---

## 🛠 环境要求

| 项目 | 要求 |
|------|------|
| Python | 3.10+ |
| 操作系统 | Windows / Linux / macOS |
| 依赖库 | PyBullet, NumPy |

---

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

**Linux/macOS:**

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. 安装依赖

```bash
pip install pybullet numpy matplotlib
```

> **注意**：在 Windows 上安装 PyBullet 时，如果提示缺少 Visual C++ 编译工具，请先安装 [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)，确保勾选 **“使用 C++ 的桌面开发”** 工作负载。

---

## 🚀 运行方法

### 第一步：查找末端链接索引（首次运行时执行）

运行 `find_ee.py`，获取机械臂末端执行器（`end_link`）的索引：

```bash
python simulation/find_ee.py
```

输出示例：

```text
关节索引 | 关节名 | 子链接名
--------------------------------
   0     | joint1      | link1
   1     | joint2      | link2
   2     | joint3      | link3
   3     | joint4      | link4
   4     | joint5      | link5
   5     | joint6      | link6
   6     | end_joint   | end_link

可能的末端链接索引（名称包含 'end'）：
  索引 6 : end_link
```

**记下 `end_link` 的索引（本例为 `6`）**，后续在主脚本中会用到。

### 第二步：运行主控制脚本

```bash
python simulation/rebot_sim.py
```

终端会提示输入目标坐标：

```text
请输入目标坐标 (x, y, z)，用逗号分隔，例如 0.15, 0.0, 0.15:
```

输入坐标后按回车，机械臂将平滑移动到目标位置。

**示例输入**：

```text
0.15, 0.0, 0.15
0.20, 0.10, 0.10
0.10, -0.10, 0.20
```

> ⚠️ **建议**：目标点在工作空间内，推荐范围：`x: 0.05~0.25`, `y: -0.15~0.15`, `z: 0.05~0.25`。

---

## 🎬 演示视频

[![点击观看演示视频](https://img.shields.io/badge/▶️-点击观看演示视频-blue)](https://github.com/XIAOHU7771/my-reBot-DevArm/releases/download/v1.0/demo.mp4)

如果视频无法加载，[点击此处直接下载观看](https://github.com/XIAOHU7771/my-reBot-DevArm/releases/download/v1.0/demo.mp4)。

---

## 📊 代码架构

```text
┌─────────────────────────────────────────────────────────────┐
│                    用户输入 (x, y, z)                        │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│              rebot_sim.py (主控制脚本)                       │
├─────────────────────────────────────────────────────────────┤
│  1. 初始化 PyBullet GUI 仿真环境                            │
│  2. 加载 reBot-DevArm URDF 模型                            │
│  3. 获取用户输入目标坐标                                    │
│  4. 调用 PyBullet IK 求解器计算关节角度                    │
│  5. 线性插值生成平滑轨迹                                    │
│  6. 驱动关节电机，步进仿真                                  │
└──────────────────────────────┬──────────────────────────────┘
                               ▼
┌─────────────────────────────────────────────────────────────┐
│              PyBullet 物理引擎                              │
├─────────────────────────────────────────────────────────────┤
│  • URDF 解析器                                             │
│  • 逆运动学数值求解                                         │
│  • 刚体动力学仿真                                           │
│  • 3D 可视化渲染                                            │
└─────────────────────────────────────────────────────────────┘
```

### 逆运动学（IK）执行流程

1. **输入**：末端目标位置 `(x, y, z)` + 当前关节角度
2. **迭代求解**（最多 500 次）：
   - 计算当前末端位置误差
   - 计算雅可比矩阵（Jacobian）
   - 更新关节角度：`θ_new = θ_old + J⁺ · Δx`
3. **输出**：满足误差阈值的 6 个关节角度

### 关键参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `end_effector_index` | 6 | 末端链接 `end_link` 的索引 |
| `maxNumIterations` | 500 | IK 最大迭代次数 |
| `residualThreshold` | 1e-5 | IK 收敛精度阈值 |
| `steps` | 100 | 插值步数（控制运动平滑度） |

---

## 🛠 故障排查

### 1. 安装 PyBullet 时提示缺少 Visual C++ 编译工具

- 下载并安装 [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/)
- 安装时务必勾选 **“使用 C++ 的桌面开发”** 工作负载
- 安装完成后**重启电脑**

### 2. IK 求解失败（`Calculate Inverse Kinematics Request failed`）

- 检查 `end_effector_index` 是否正确（重新运行 `find_ee.py` 确认）
- 目标点可能超出工作空间，尝试更保守的值，如 `[0.15, 0.0, 0.15]`
- 增大 `maxNumIterations` 参数（如 1000）
- 

---

## 📝 许可证

本项目基于 reBot-DevArm 开源项目，遵循 [CERN-OHL-W-2.0](https://ohwr.org/cern_ohl_w_v2) 许可证。

---

## 🤝 致谢

- [reBot-DevArm](https://github.com/Seeed-Projects/reBot-DevArm) - 开源硬件项目
- [reBotArm_control_py](https://github.com/vectorBH6/reBotArm_control_py) - 运动学控制库参考
- [PyBullet](https://pybullet.org/) - 物理仿真引擎
