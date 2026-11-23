# LIMO 自主泊车仿真

**项目版本 v0.6.0**

## 项目做了什么

在 Gazebo 中搭建无标线停车场，并接入放大后的 AgileX LIMO 阿克曼转向模型。任务由两圈组成。第一圈沿主路巡检、在线建图并返回起点。第二圈按白、蓝、红、黄、绿五个区域依次检查，最后在绿框内垂直倒车入库。

| 区域 | 演示内容 |
| --- | --- |
| 白框 | 激光观测到静态障碍车，LIMO 沿主路绕行。 |
| 蓝框 | 入口被两侧车辆压窄，车辆在进入前拒绝泊车。 |
| 红框 | 一名行人在车位入口两辆车之间往返，车辆跟踪到持续运动后跳过。 |
| 黄框 | 斜停车辆压缩倒车空间，整段车身扫掠检查未通过。 |
| 绿框 | 激光测得可用空位，车辆完成垂直倒入和停稳验证。 |

核心对照是在同一份感知地图上，分别检查**只看最终车身是否放得下**和**整段倒车是否有足够净空**。仿真真值用于独立核验。任务运行期间，运动保护节点使用实时激光和里程计检查制动距离。

## 软件与仿真设备

| 组件 | 用途 |
| --- | --- |
| Ubuntu 24.04 LTS 与 Bash | 运行 ROS 和仿真脚本。交付复现使用 Windows 的 WSL 2 Ubuntu，架构为 x86_64。 |
| ROS 2 Jazzy 与 `ros_gz` | 发布与桥接激光、里程计、坐标变换、速度指令和仿真时钟。 |
| Gazebo Harmonic | 加载场景、车辆模型、行人动画与传感器，运行车辆动力学。已验收环境的 Gazebo Sim 版本为 8.15.0。 |
| SLAM Toolbox、Nav2、RViz2 | 在线建图与定位、路径规划，以及地图、回波和规划路径的可视化。 |
| Python 3.12、NumPy | 占据网格、泊车判断、轨迹跟踪、运动保护与实验验收。 |
| VS Code | 浏览源码并运行工作区任务。 |

车辆模型包含车身、轮组和四轮驱动的前轮阿克曼转向，以及 360° 激光雷达、前后近地激光、RGB 相机、IMU、轮速里程计和底盘接触检测。车身与轮组网格来自 AgileX LIMO 的 Humble 分支，本项目在 Jazzy 和 Harmonic 中加载适配后的 SDF、URDF 与接口配置。 [模型来源与许可](parking_lot_sim/models/limo/README.md)

## 第一次运行

1. **准备 Ubuntu 24.04。** Ubuntu 用户直接打开终端。Windows 用户在管理员 PowerShell 中运行下列命令，完成安装后打开 Ubuntu 终端。macOS 或其他系统可使用具备图形渲染能力的 Ubuntu 24.04 虚拟机，或连接到符合上述要求的 Ubuntu 电脑。 [WSL 安装指南](https://learn.microsoft.com/windows/wsl/install) · [WSL 图形应用指南](https://learn.microsoft.com/windows/wsl/tutorials/gui-apps)

   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. **安装依赖。** 首次使用 ROS 的组员先按照官方指南配置 ROS 2 的 apt 软件源。随后在 Ubuntu 终端安装项目依赖。已有 Jazzy 环境的组员可直接检查或补齐下列软件包。 [ROS 2 Jazzy 安装指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) · [Jazzy 与 Gazebo Harmonic 配套说明](https://gazebosim.org/docs/harmonic/ros_installation/)

   ```bash
   sudo apt update
   sudo apt install ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-navigation2 \
     ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rviz2 \
     ros-jazzy-robot-state-publisher python3-numpy unzip
   ```

3. **解压并启动。** 完整解压 ZIP，在 Ubuntu 终端进入解压后的 `LIMO_Autonomous_Parking` 目录，也就是有本 README 的目录，然后运行下列命令。场景和模型都随包提供，启动脚本会加载 Jazzy 环境，可以直接从源码目录运行。

   ```bash
   cd parking_lot_sim
   bash run_autonomous.sh
   ```

   默认打开 Gazebo 和 RViz2 并启用红框行人。任务结束后可以继续查看结果。退出时在原终端按 **Ctrl+C**。切换运行方式前先退出上一轮仿真。

   Windows 用户也可以在 Ubuntu 终端用 `cd` 加上带引号的解压目录路径。Windows 的 C 盘在 WSL 中对应 `/mnt/c`，其他盘符遵循同样的规则。

| 运行方式 | 在 `parking_lot_sim/` 中执行 |
| --- | --- |
| 默认两圈，打开 Gazebo 和 RViz2 | `bash run_autonomous.sh` |
| 无界面运行两圈 | `bash run_autonomous.sh gui:=false rviz:=false` |
| 静态场景对照 | `bash run_autonomous.sh pedestrians:=false` |
| 只打开停车场和车辆模型 | `bash run.sh` |

无界面运行也使用 Gazebo 的激光渲染。若无法启动，先检查 Ubuntu 版本、Jazzy 软件源及所用电脑的图形驱动或虚拟机图形加速。可在 Ubuntu 终端执行以下环境检查。

```bash
source /opt/ros/jazzy/setup.bash
python3 -B scripts/check_runtime.py
gz sim --version
```

## 在 VS Code 打开源码

在项目根目录打开 `LIMO_Parking.code-workspace`。如果在 Windows 上使用 WSL，先安装 VS Code 的 **WSL** 扩展，在项目根目录的 Ubuntu 终端执行下列命令。然后通过 **Terminal → Run Task** 运行环境检查、自主泊车、单元测试和随包记录验收。命令行和工作区任务调用同一份源码。 [VS Code 的 WSL 工作区指南](https://code.visualstudio.com/docs/remote/wsl-tutorial)

```bash
code LIMO_Parking.code-workspace
```

## 项目目录

| 位置 | 包含内容 |
| --- | --- |
| `parking_lot_sim/worlds/`、`models/`、`urdf/` | 停车场与各测试起点、LIMO 网格和车载传感器、行人模型与动画。 |
| `parking_lot_sim/scripts/`、`launch/`、`config/` | 自主任务、地图与泊车判定、轨迹跟踪和运动保护、ROS 启动与配置。 |
| `parking_lot_sim/tests/` | 车身几何、激光可见性、停车路径、循迹和指令安全测试。 |
| `parking_lot_sim/results/verified_two_lap_entrance/` | 当前版本的首圈和最终地图、原始轨迹、逐站决策以及独立验收。 |
| `parking_lot_sim/docs/` | 算法、传感器、实验方法和历史对照记录。 |

## 复核实验

当前版本的 Gazebo 记录包含 **约 9.8 厘米的首圈回点误差、约 41.4 厘米的最小采样静态净空、零底盘接触记录**。验收确认整车在绿框内停稳。每次新运行会在 `parking_lot_sim/results/` 中创建一个时间戳目录，并在终端显示目录名。运行到终端出现 `parked` 和该目录下的 `result.json` 后，即可验收这次运行。 [随包验收记录](parking_lot_sim/results/verified_two_lap_entrance/evaluation.json) · [算法和实验说明](parking_lot_sim/docs/AUTONOMOUS_PARKING.md)

在 `parking_lot_sim/` 下运行以下命令可检查场景、执行 58 项单元测试并独立复核随包记录。要验收新运行，将最后一条命令的目录替换为该次运行的目录。

```bash
python3 -B scripts/validate_scenarios.py
PYTHONPATH=./scripts python3 -B -m unittest discover -s tests
python3 -B scripts/evaluate_run.py results/verified_two_lap_entrance --check-demonstration
```

项目代码的许可与参考资料见 [MIT 许可](parking_lot_sim/LICENSE) 和 [技术文档](parking_lot_sim/docs/AUTONOMOUS_PARKING.md)。
