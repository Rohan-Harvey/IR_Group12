# LIMO Autonomous Parking v0.7.2

An Ackermann-steered LIMO surveys an unmarked car park in Gazebo using lidar and wheel odometry. On its first lap it builds a map with SLAM Toolbox, returns to the entrance and saves the map. At the start of the second lap, discrete Markov localisation initialises a particle filter. The vehicle uses the resulting pose estimate during the parking inspection.

| Test scenario | What the vehicle checks |
| --- | --- |
| 1 | The space between two misparked cars. If the vehicle's swept footprint intersects an obstacle, it skips the bay and continues along the aisle. |
| 2 | A restricted entrance. The vehicle rejects the bay when the entrance has insufficient clearance. |
| 3 | A moving obstacle crossing the entrance. The vehicle skips the bay and RViz highlights the crossing area and tracked lidar returns. |
| 4 | A wide entrance with two inward-angled cars that narrow the rear of the bay. The vehicle checks rear clearance and the space the body needs throughout the reversing manoeuvre. |
| 5 | A clear bay. The vehicle reverses into the space and checks its final position, heading and standstill. |

## Run the simulation

Use Ubuntu 24.04 (native or WSL 2), ROS 2 Jazzy and Gazebo Harmonic. With the Jazzy package repository configured, install the dependencies:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-base ros-jazzy-ros-gz ros-jazzy-navigation2   ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox ros-jazzy-rviz2   ros-jazzy-robot-state-publisher python3-numpy
```

Open an Ubuntu terminal in the extracted `LIMO_Autonomous_Parking` directory and run:

```bash
cd parking_lot_sim
bash run_autonomous.sh
```

Gazebo shows the vehicle and car park. RViz shows the map, lidar returns and parking decisions. The terminal reports `parked` when the vehicle has settled in the final bay. The launcher prints the output directory for each run. Press **Ctrl+C** to exit.

## Verify a run

From `parking_lot_sim`, run the scene checks and unit tests:

```bash
python3 -B scripts/validate_scenarios.py
PYTHONPATH=./scripts python3 -B -m unittest discover -s tests
```

To check the full demonstration, replace `<run-directory>` with the output path printed by the launcher:

```bash
python3 -B scripts/evaluate_run.py "<run-directory>" --check-demonstration
```
