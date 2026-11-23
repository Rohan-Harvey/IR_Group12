# LIMO model resources

The body and wheel DAE files are unchanged copies of the official AgileX LIMO
ROS 2 repository, humble branch, limo_car/meshes:
https://github.com/agilexrobotics/limo_ros2

The corresponding official Xacro files and vendor readme are retained in
source/. The official model's proportions and original mesh materials are
preserved. The robot is uniformly enlarged to a 2.12 m straight-ahead width.

The SDF integration, collision envelopes, dynamics, ROS sensor interfaces and
Harmonic drive plugin are project adaptations, not an official AgileX Jazzy
release or a claim that this is an unmodified physical LIMO configuration.

The upstream [`limo_car` manifest](https://github.com/agilexrobotics/limo_ros2/blob/humble/limo_car/package.xml)
declares Apache-2.0. The [license text](LICENSE.apache-2.0) is included alongside
the vendor resources. The project MIT license applies to the project code.
