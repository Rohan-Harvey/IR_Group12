# Validation record

The following checks run with Python without a Gazebo or ROS installation.

| Check | Result |
| --- | --- |
| Well-formed world XML, unique model/geometry names and finite dimensions | Passed |
| Both main world filenames contain identical unmarked geometry | Passed |
| 47 parked sedans with no overlapping conservative envelopes | Passed |
| Parked cars retain the configured minimum separation | Passed |
| Full robot footprint matches the parked sedan envelope, 4.62 × 2.12 m | Passed |
| Four powered wheel joints and two front steering joints | Passed |
| Positive link masses/inertias; matching wheel and steering names in URDF | Passed |
| LiDAR, camera, IMU, contact sensor and TF definitions | Passed |
| Six walking routes and bundled COLLADA rigs | Passed |
| Six scenario starts are clear of static scenery | Passed |
| S1 final footprint collides with inward-angled cars | Passed |
| S2 final placement and reference path fit; pedestrian initially occupies sweep | Passed |
| S3 opening narrower than the physical chassis | Passed |
| S4 diagonal pose fits but fails heading tolerance; perpendicular goal collides | Passed |
| S5 final placement fits; closed blocked curve certifies no planar entry | Passed |
| S6 reverse arc stays clear and ends perpendicular to the curb | Passed |
| Reference path curvature satisfies steering limits | Passed |
| Extra footprint padding covers geometric interpolation between arc samples | Passed |
| Offline evaluator rejects incorrect final heading and blocked entry | Passed |
| Default layout has one initially clear accepted case, S6 | Passed |

The overview, six case panels and robot drawing were visually inspected. The
pedestrian mesh was inspected at multiple gait phases. These are generated
geometry previews, not screenshots from a running Gazebo simulator.

Gazebo, ROS 2, libsdformat and colcon are unavailable here. The package has
not been built or run in Gazebo. Live actor import, wheel traction, steering
tracking, odometry, sensor publication, ROS bridges and actual parking
completion remain untested. The reference arcs establish a collision-free
kinematic example, not verified execution. Pedestrians have no contact physics.
