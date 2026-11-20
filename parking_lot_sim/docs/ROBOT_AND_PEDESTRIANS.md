# Car-sized robot and walking people

The robot now uses the official AgileX LIMO body and wheel meshes, with
uniform enlargement and project-specific Harmonic dynamics/interfaces.

The own-robot body and wheel visuals use visibility bit 2; its GPU LiDAR
uses mask bit 1. Thus its official visual mesh is excluded from its own
laser measurements, while the GUI/RGB camera and external scene remain
visible. This is a deliberate own-body exclusion simplification, not an
optical self-occlusion model. The scan plane intersects the body mesh, so
the unmasked version could rasterize the robot itself as nearby obstacles.

- Straight-ahead footprint: approximately 3.00 × 2.12 m.
- Conservative steering envelope width: 2.56 m.
- Wheelbase: 1.865 m; kingpin track: 1.306 m; wheel radius: 0.449 m.
- Virtual steering limit: 0.45 rad; physical steering joints: ±0.5236 rad.
- Four powered wheels, front Ackermann steering; it cannot spin in place.

The official mesh files, colors and proportions are preserved. Body, wheel,
collision, mass, inertia and matching ROS frames were adapted together. Sensor
mounts follow the official model's scaled mounting offsets. Sensor ranges and
ROS topic names retain this scene's interfaces. Gazebo operation is not yet
runtime-verified by these model checks. Original car-size scenario conclusions
require a LIMO-specific assessment.

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
| `/bumper_contacts` | `ros_gz_interfaces/msg/Contacts` | Physical chassis contacts |
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

One pedestrian walks back and forth across the red S6 bay mouth along
\(y=12.65\) m between \(x=10.75\) m and \(x=17.25\) m. The actor uses the
bundled green COLLADA skin and articulated walking animation. The actor
appears to the camera and GPU LiDAR and does not generate contact forces.

Gazebo/ROS runtime behavior remains untested in the creation workspace.
Official references: [Ackermann controller](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1AckermannSteering.html),
[actors](https://gazebosim.org/docs/harmonic/actors/), and
[ROS bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/README.md).
