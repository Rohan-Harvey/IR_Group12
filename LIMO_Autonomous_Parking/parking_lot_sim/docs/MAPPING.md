# Laser mapping

For the normal autonomous mission, use `bash run_autonomous.sh` and follow
[Autonomous mapping](AUTONOMOUS_MAPPING.md). The keyboard procedure below is
retained only as an optional debugging mode.

The mapping launch attaches to a running Gazebo world. It does not spawn a
second world or change the robot, parked cars or pedestrians. It consumes
laser measurements and wheel odometry, not the scene ground-truth JSON or
the original scenario reference paths. The map starts from observations at
the robot's current position; it is not a preloaded parking-lot drawing.

## Start from the current keyboard-driving session

Keep Gazebo running. Stop the previous standalone `parameter_bridge` with
Ctrl+C so there is one bridge. The keyboard terminal can stay open; press
Space before switching windows. Stop a scrolling `ros2 topic echo` with
Ctrl+C if it is no longer needed.

In an Ubuntu terminal, from this `parking_lot_sim` directory:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch launch/mapping.launch.py
```

No project build or installation is needed for this direct source-tree
launch. For an installed package the equivalent command is:

```bash
ros2 launch parking_lot_sim mapping.launch.py
```

The launch starts the control/sensor/time bridge, robot transforms, the
installed slam_toolbox asynchronous mapping launch, the map adapter, and RViz. All nodes use
Gazebo simulation time. Leave that terminal running. If a required ROS
package is missing, install only the missing component:

```bash
sudo apt update
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-rviz2 ros-jazzy-robot-state-publisher
```

If Gazebo was started with the complete `parking_lot.launch.py` ROS launch
instead of `bash run.sh`, keep its existing bridge and state publisher and
use `ros2 launch launch/mapping.launch.py bridge:=false state_publisher:=false`.
An optional `rviz:=false` starts mapping without opening its display.

## Drive and read the map

Use the existing keyboard terminal. Alternatively, in another Ubuntu
terminal in the project directory:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/teleop_robot.py
```

Keep keyboard focus in this terminal. W/S move forward/reverse; A/D steer
while moving forward; Z/C steer while reversing; Space stops.

In RViz, red dots are current laser returns. Black cells are occupied,
white cells are observed free space, and grey cells are unobserved or
uncertain. The coloured axes indicate the robot position and orientation;
the red axis points forward. A model with detailed wheel meshes is not
needed in this top-down mapping view.

The default display now renders `/map_preview` as colored points rather than
an indexed map texture, avoiding the shader error reported on the user's WSL
graphics setup. Point count is capped for display performance; the raw SLAM
grid and full-resolution navigation grid are unaffected. The optional native
map texture is disabled by default. `/navigation_map` adds only unknown border
cells around raw `/map`, so an entrance pose close to the raw map boundary
can be handled by Nav2. Map saving continues to use the original `/map`.

Drive a short distance through a visibly clear aisle first and watch the
map extend. Then cover the connected aisles slowly and revisit an earlier
area so scan matching and loop closure have overlapping observations.
This is a car-like robot: use forward/reverse arcs rather than trying to
rotate in place. Move only where the body and its steered wheels have room.

The initial grid has 0.05 m cells and updates about once per second of
simulation time. Cell size is not localisation or obstacle-boundary
accuracy. Moving people may leave transient returns or traces; do not
interpret the first map as proof that a tight parking manoeuvre is safe.

## Save a map after observing enough of the lot

Keep mapping running and stop the robot. The installed map saver can save
an occupancy map. In a separate Ubuntu terminal:

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/parking_maps
ros2 run nav2_map_server map_saver_cli -f ~/parking_maps/parking_lot --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
```

If `nav2_map_server` is missing, install `ros-jazzy-nav2-map-server` first.
The PGM image and YAML metadata preserve the occupancy grid. They do not
preserve a resumable SLAM pose graph; saving that is a separate operation.
Otherwise the live map is lost when mapping is stopped.

## Frame ownership

- slam_toolbox publishes `map` to `odom`.
- The Gazebo wheel controller publishes `odom` to `rear_axle`, using rear-wheel odometry.
- robot_state_publisher publishes the body, wheel and sensor frames from
  the existing URDF, including `rear_axle` to `base_footprint` and onwards to `lidar_link`.
- The scan message uses `lidar_link`; Gazebo's clock is bridged to `/clock`.

Starting mapping after driving is supported: it starts a new map around
the current scan using the current odometry. Do not reset or restart
Gazebo while leaving this mapping process running; restart mapping after
a world reset to avoid mixing different simulation timelines.

The launch and configuration have been checked statically on the Windows
host. Sensor visibility, live transforms and SLAM quality must be observed
in the running Ubuntu/Gazebo session; no completed map is bundled here.

## References

- [Nav2 mapping setup](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/sensors/mapping_localization/)
- [slam_toolbox launch](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py)
- [slam_toolbox parameters](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/config/mapper_params_online_async.yaml)
