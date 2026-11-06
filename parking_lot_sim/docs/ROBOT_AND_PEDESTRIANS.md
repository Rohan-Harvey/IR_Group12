# Car-sized robot and walking people

The robot is a low sensor platform with a **4.62 × 2.12 m footprint**, exactly
the parked sedan envelope. Its four wheels are powered. Two front steering
joints implement Ackermann steering with a 2.75 m wheelbase, 1.58 m track,
0.32 m wheel radius, and 0.55 rad virtual steering limit. It cannot turn in
place. Physical steering joints are limited to ±0.65 rad. Masses, inertias,
controller geometry and the matching ROS URDF were updated together.

The platform remains lower than a car so its horizontal LiDAR can observe
car body boundaries. The robot's height is not a replica of sedan bodywork;
the car-sized footprint governs the parking tests.

## Keyboard control

Launch through ROS as described in the README, then in another sourced terminal:

```bash
ros2 run parking_lot_sim teleop_robot
```

| Key | Action |
| --- | --- |
| W | Forward at 0.4 m/s |
| S | Reverse at 0.4 m/s |
| A / D | Forward while steering left / right |
| Z / C | Reverse while steering left / right |
| Space or X | Stop |
| Q or Ctrl+C | Stop and exit |

Hold or repeat a movement key. The script sends zero velocity after 0.4 seconds
without keyboard input. It has no obstacle avoidance or autonomous parking.
The forward/reverse turning keys use gentle curvature suitable for the steering
limits. Other controllers can publish a linear speed and yaw rate on `/cmd_vel`.

## Sensor and state topics

| ROS topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Forward/reverse speed and yaw-rate commands |
| `/scan` | `sensor_msgs/msg/LaserScan` | 360° GPU LiDAR, 720 samples, 10 Hz, 30 m range |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Front RGB camera, 640×480, 15 Hz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/imu` | `sensor_msgs/msg/Imu` | IMU at 100 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | Drive-controller odometry, 30 Hz |
| `/joint_states` | `sensor_msgs/msg/JointState` | Four wheel joints and two steering joints |
| `/tf` and `/tf_static` | `tf2_msgs/msg/TFMessage` | Odometry, body and sensor transforms |
| `/bumper_contacts` | `ros_gz_interfaces/msg/Contacts` | Physical front bumper contacts |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |

The odometry frame starts at the vehicle's initial state. It is not world
ground truth. The sensor frames are `lidar_link`, `imu_link` and
`camera_optical_frame`. Use `use_sim_time: true` for project nodes.

Example checks on Ubuntu:

```bash
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 topic echo /imu --once --qos-reliability best_effort
ros2 topic hz /odom
```

For the camera, install `ros-jazzy-rqt-image-view` if needed, then:

```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

Five pedestrians walk along the outer sidewalks. The sixth crosses the S2
reverse-entry region. All have original bundled COLLADA skins and articulated
walking animation. They are visual/GPU-sensor obstacles without physical
collision responses; evaluate sensor-based stopping and minimum distance.

Gazebo/ROS runtime behavior remains untested in the creation workspace.
Official references: [Ackermann controller](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html),
[actors](https://gazebosim.org/docs/harmonic/actors/), and
[ROS bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md).
