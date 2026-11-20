# Validation record (updated 2026-11-19)

This section supersedes the historical checks below. The current robot is
the enlarged LIMO (approximately 3.00 m long and 2.12 m straight-ahead width),
with a wider conservative steered-wheel envelope. Original sedan-sized
parking references are not certified LIMO parking results.

Current executed checks:

- `python -m unittest discover -s tests -v`: all 40 offline tests passed.
  These cover survey continuity, rear-axle motion, padded route clearance,
  steering and speed constraints, freshness holds, node construction,
  navigation rejection/timeout/cancellation, late action replies, endpoint
  approach and map-saver startup failure. The new tests reproduce a tight
  initial-map entrance deadlock and verify that unknown-only map padding
  permits a first target, preserves every measured cell/world coordinate,
  and bounds the separate point-cloud display data. Integration tests use
  ROS-shaped message stubs and do not execute the Nav2 planner.
  Direction regressions additionally verify forward-only startup defaults,
  rejection of negative survey commands, permission for reverse return,
  rejection of forward return commands, freshness of the direction signal,
  clearing of commands across phase changes, two acknowledged parameter
  changes before movement, and fail-closed parameter errors/timeouts. Wheel
  axes are checked statically; this does not establish live wheel traction.
  Both inflation radius and a nonzero 8-bit cost at the circumscribed radius
  are checked using the documented Nav2 exponential decay.
  Self-visibility tests verify all eight worlds and newly generated robots
  exclude their own meshes from GPU LiDAR while preserving external scene
  visibility, plus detection of a synthetic own-body range cluster by the
  read-only diagnostic summary.
- `python scripts/validate_world.py`: passed the supplied static-world,
  car separation, robot, sensor, TF and bundled actor-resource checks.
- `python -m compileall -q scripts launch tests`: passed.
- Both entry scripts use LF line endings. Shell execution and `bash -n`
  could not be checked: Git Bash was also blocked by Windows sandbox access
  controls (`NtCreateDirectoryObject`, access denied).

ROS 2, Nav2 and Gazebo were not executed in this Windows editing session.
WSL access returned `Wsl/E_ACCESSDENIED`. The user's screenshots and attached
log confirm Gazebo and RViz startup, SLAM map publication, Nav2 lifecycle
activation, and an entrance hold with costmap-boundary warnings in 0.4.1.
Version 0.4.2 addresses this using a separate unknown-padded navigation map;
the original SLAM map and saved-map topic stay intact. A point-cloud display
avoids the reported RViz indexed-texture shader, helper GUI panels are
shrunk, and inflation now covers the rear-axle footprint's outer radius.
The revised live startup, driving and complete mission remain unverified.
The user's subsequent 0.4.2 screenshots show the vehicle moving backwards
from the entrance, with a curved Nav2 plan and a remaining inflation warning.
Version 0.4.3 starts with DUBIN and reversing disabled, changes navigation
parameters only while stopped and after both services acknowledge, and
enforces the active phase's direction at the final velocity publisher.
The inflation cost decay now stays nonzero at the footprint's outer radius.
Actual 0.4.3 motion and live parameter switching still require validation.

The user's 0.4.3 screenshot shows repeated `no valid path found` from the
entrance to (8.10, 0.67). Inspection of the actual COLLADA mesh found a further
sensor defect: the horizontal scan plane intersects the robot's own body,
which had the default all-bits visibility. An independent two-sided triangle
plane-section check found 647/720 geometric own-body ray intersections,
including all 120 sampled rays within +/-30 degrees forward. The forward
ray intersection is approximately 0.257 m away. This is offline geometric
evidence, not a measured Gazebo scan; rendering back-face culling and other
renderer details were not emulated.

Version 0.4.4 places all own-robot visuals on bit 2 and uses LiDAR mask bit 1.
External scenery retains its default visibility. This excludes own-body
returns without changing camera visibility, physics, inertia, joint axes,
laser pose/range, or occupancy values. It deliberately simplifies physical
self-occlusion and must not be presented as a complete sensor optical model.
An actual scan and successful forward plan remain unverified. A read-only
diagnostic script now captures raw scans, maps, commands, mission status and
the map-to-rear-axle transform for an evidence-based follow-up if needed.
The startup regression tests use ROS-shaped stubs rather than a running
ROS graph. The static route check uses scene truth only for independent
validation; the live mission does not load obstacle ground truth.

## Historical checks for the original vehicle

The entries below were recorded for the former 4.62 m vehicle. They are
retained as scene-design history, and do not establish current LIMO parking
feasibility or current live navigation performance.

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
