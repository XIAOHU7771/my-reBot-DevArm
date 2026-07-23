# my-reBot-DevArm
基于 PyBullet 的 reBot-DevArm 六轴机械臂虚拟数字孪生仿真项目。

## 📖 项目概述
仓库包含两套**相互独立**的仿真工程，分别使用不同的urdf模型：
1. **simulation**：基础笛卡尔空间点位运动仿真，用于机械臂逆运动学与点位控制学习。
2. **simulation2**：平行夹爪高级抓取仿真，集成虚拟触觉传感、对称开合、自适应力控、滑移补偿算法。


## ✨ 功能列表
### simulation｜基础点位运动仿真
- 加载 reBot-DevArm 机械臂 URDF 模型
- 终端交互式输入末端 TCP 笛卡尔坐标
- PyBullet 内置数值逆运动学 IK 求解
- 线性插值平滑轨迹生成
- GUI 3D可视化仿真

### simulation2｜夹爪力控抓取仿真
- 平行夹爪「指心不变」对称开合控制
- 双路力采集：接触点正压力、关节反作用力
- 自适应双指力均衡夹持控制器
- 刚性铁块 / 柔性海绵抓取对照实验
- 物体滑移检测与自动加紧补偿
- 力时序数据导出 CSV、实时绘制力曲线

## 📂 仓库目录结构
my-reBot-DevArm/
├── simulation/ # 独立工程 1：基础 TCP 点位运动仿真
│ ├── find_ee.py # 查询关节、Link 索引工具
│ └── rebot_sim.py # 笛卡尔坐标控制主程序
├── simulation2/ # 独立工程 2：夹爪力传感 & 力控抓取仿真
│ ├── 1.Force_Sensor_Simulation.py # 虚拟触觉仿真、力数据记录
│ ├── 3.adaptive_force_control_grasp.py # 自适应力控抓取实验
│ ├── 4.slip_compensation_test.py # 滑移对冲补偿实验
│ ├── utils/ # 夹爪控制、力传感工具包
│ └── force_data/ # 导出 CSV、力曲线图片
├── urdf/ # 机械臂 URDF 模型文件
├── requirements.txt
└── README.md
plaintext

## 🛠 环境依赖
- Python ≥ 3.10
- 支持 Windows / Linux / macOS
- 基础依赖：`pybullet`, `numpy`, `matplotlib`

## 📦 安装步骤
### 1. 克隆代码仓库
```bash
git clone https://github.com/XIAOHU7771/my-reBot-DevArm.git
cd my-reBot-DevArm
2. 创建并激活虚拟环境
Windows PowerShell
powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
Linux / macOS
bash
python3 -m venv .venv
source .venv/bin/activate
3. 安装依赖包
bash
# 全局基础依赖
pip install pybullet numpy matplotlib

# simulation2模块额外依赖
pip install -r simulation2/requirements.txt
⚠️ Windows 安装 PyBullet 若出现编译报错，请安装 Microsoft C++ Build Tools，勾选「使用 C++ 的桌面开发」工作负载。
🚀 运行指南
simulation｜基础点位运动仿真
查询末端 Link 索引（首次运行必须执行）
bash
python simulation/find_ee.py
启动笛卡尔坐标控制程序
bash
python simulation/rebot_sim.py
终端提示输入目标坐标，格式示例：
plaintext
0.15, 0.0, 0.15
推荐工作区间：x:0.05~0.25, y:-0.15~0.15, z:0.05~0.25
simulation2｜力控抓取仿真
bash
cd simulation2

# 虚拟力传感器仿真，记录力曲线
python 1.Force_Sensor_Simulation.py

# 启用力控抓取
python 2.adaptive_force_control_grasp.py --force-control

# 无力控对照实验
python 2.adaptive_force_control_grasp.py --no-force-control

# 开启滑移自动补偿实验
python 3.slip_compensation_test.py

📀 Release 资源说明
Release 页面附带资源：
自适应力控抓取仿真演示视频
基础点位运动视频

📄 许可证
本项目基于 reBot-DevArm 开源硬件项目，遵循 CERN-OHL-W-2.0 开源协议。

🤝 致谢
reBot-DevArm 开源机械臂硬件项目
PyBullet 物理仿真引擎