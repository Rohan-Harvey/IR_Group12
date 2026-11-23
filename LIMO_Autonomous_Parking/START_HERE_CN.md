# IR 无标线停车仿真

项目使用 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic。进入解压后的项目根目录，运行以下命令。环境安装和 VS Code 操作见 [项目 README](README.md)。

```bash
cd parking_lot_sim
bash run_autonomous.sh
```

默认任务先循迹环绕一圈，沿入口连接段返回起始点，停稳后保存第一圈地图。车辆随后再次出发，依次经过白、蓝、红、黄、绿五处。白框沿主路绕行。第二圈用激光检查各处的可用空间、入口和整段倒车路径。红框唯一的行人在两车之间往返。RViz 显示 SLAM 地图、激光回波和当前路径。

| 运行方式 | 命令 |
| --- | --- |
| 先跑静态环境 | `bash run_autonomous.sh pedestrians:=false` |
| 从绿框 S1 附近起步核验倒车 | `bash run_autonomous.sh scenario:=S1 pedestrians:=false` |
| 保持后台运行 | `bash run_autonomous.sh gui:=false rviz:=false` |
| 只打开场景 | `bash run.sh` |

切换运行方式前，在原终端按 **Ctrl+C**。项目不需要重新编译。缺少依赖时在 Ubuntu 中安装。

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-robot-state-publisher python3-numpy
```

## 查看结果

每次任务会在项目的 `results/` 下创建一个带时间戳的目录。终端会打印目录位置和任务状态。

- `result.json` 记录第一圈是否完成、逐站结果、是否停稳，以及估计的最终位置和朝向误差。
- `decisions.jsonl` 记录首圈回点、逐站测量、最终位姿与完整倒车路径的对照及换向事件。
- `run.csv` 记录速度指令、前轮角度、IMU、里程计和 Gazebo 真值。
- `first_lap_map.pgm` 和 `first_lap_map.yaml` 保存首圈回点时的地图，`parking_map.pgm` 和 `parking_map.yaml` 保存任务结束时的地图。

用 `python3 scripts/evaluate_run.py results/<运行目录名> --check-demonstration` 核验默认任务的回点、逐站判断和绿框泊车结果，并计算基于 Gazebo 记录的最终误差和采样静态间隙。算法、传感器及验证边界见 [实现与实验说明](parking_lot_sim/docs/AUTONOMOUS_PARKING.md)。
