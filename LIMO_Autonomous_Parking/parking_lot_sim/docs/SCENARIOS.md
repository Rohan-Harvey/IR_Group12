# Scenario geometry and evaluation

The scene geometry, goals and routes in this document are retained from the
original project. Its numerical feasibility conclusions were derived for the
previous 4.62 × 2.12 m robot. This copy uses a uniformly enlarged official-mesh
LIMO (about 3.00 × 2.12 m straight ahead); recompute those conclusions and
reference manoeuvres before using them as LIMO evaluation labels.

The single combined lot provides six identifiable cases. The case names in
this document and in the JSON are evaluation references; there are no painted
bays, goal markers or automatic target-selection rules in the world.

| Case | Goal center x/y (m) | Required heading | Reference result |
| --- | --- | --- | --- |
| S1 | −14.0, +14.6 | −90° | Perpendicular footprint intersects inward-angled neighbors |
| S2 | 0.0, +14.6 | −90° | Static footprint and path fit, person initially intersects sweep |
| S3 | −14.0, −14.6 | +90° | 1.70 m opening < vehicle width |
| S4 | 0.0, −13.0 | +90° | +55° pose fits, but its 35° heading error fails |
| S5 | +14.0, −14.6 | +90° | Final footprint fits, approach throat cannot pass chassis |
| S6 | +14.0, +14.6 | −90° | Large clear space with a perpendicular reverse arc |

Headings are measured counter-clockwise from world +x. The final front points
toward the aisle, with the rear toward the curb. A ±10° heading tolerance is
fixed for evaluation. A 0.15 m safety margin surrounds the full 4.62 × 2.12 m
vehicle envelope. The physical chassis is 4.50 × 1.84 m; mirrors and bumpers
extend the envelope to match the parked sedan envelope exactly.

## S1 — inward angles

The flanking cars are at 60° and 120° and point inward. Their entrance looks
wide, but their corners reduce the interior clearance. The required full
perpendicular footprint, including the margin, collides with them. Entrance
width alone must not imply that a space is usable.

## S2 — usable geometry, dynamic blockage

The 7.20 m opening admits the reference reverse arc. The green pedestrian
`pedestrian_S2_blocker` initially stands at (2.5, 11.2), on the swept area, and
walks back and forth to (−3.5, 11.2). The robot must stop or wait while its
actual planned path is occupied, then reassess using current sensing.

There is deliberately no rule permanently rejecting S2. S6 is the one clear
choice at the initial snapshot; S2 can later become safe. Exact blocking times
depend on the vehicle's trajectory and arrival time. For evaluation, use the
actual actor poses and planned robot footprint history, rather than assuming
the initial snapshot stays blocked forever.

Walking actors have visual meshes detectable by cameras and GPU sensors, but
no contact response. A robot can physically pass through them in Gazebo.
Sensor-based stopping and pedestrian-distance checks must record that as an
unsafe event; bumper contacts cannot detect actor impacts.

## S3 — narrow gap

The 1.70 m opening is narrower than the 2.12 m full envelope and even narrower
than the 1.84 m chassis. It is unsuitable before accounting for uncertainty or
turning. The ordinary gaps between background cars are also too narrow for
the margin-expanded vehicle.

## S4 — wrong final orientation

The two flanking cars create a pocket aligned at +55°. The matching diagonal
robot pose fits with the margin, while the perpendicular reference pose at
+90° collides. A collision-free final pose still fails if its heading is 35°
from the required orientation. This separates orientation correctness from
collision correctness.

The offline evaluator applies the heading gate to every case. For example,
S6 at (14, 14.6, −55°) is rejected even though it lies in a large space.

## S5 — placement fits, approach does not

The final pocket is 3.40 m wide and can contain the complete car footprint.
Two cars parked across its approach leave a 1.75 m throat. The side cars and
the low back wall enclose the pocket, leaving no route around that throat.

The validator extracts actual SDF collision boxes at chassis height. It checks
a closed curve of forbidden chassis-center positions around the target after
inflating those boxes by the chassis' inscribed-disc radius (0.92 m). Sampling
plus a distance-function Lipschitz bound verifies coverage between samples.
The target is inside that closed blocked curve and the start is outside.
Therefore a planar vehicle cannot reach the pocket, regardless of steering
or additional forward/reverse shunts. This excludes climbing obstacles or
moving parked cars. It is stronger than merely failing one proposed path.

## S6 — clear reverse park

The 8.00 m opening provides a clear quarter-circle reverse manoeuvre with a
5.50 m rear-axle radius and a 2.75 m wheelbase. Model-center waypoints are saved
in `config/reference_paths/S6_reverse.csv`. The final model center is
(14, 14.6), heading −90°, perpendicular to the north curb.

The reference CSV starts and ends at rest, reverses at up to 0.4 m/s, and uses
a nominal acceleration magnitude of 0.4 m/s². Steering is within the 0.55 rad
virtual limit and the 0.65 rad physical front-joint limits. It is a kinematic
reference; actuator response and tracking still need a simulator run.

The path is checked against conservative parked-car envelopes, the planter,
curbs, island and east wall at 0.25° intervals. An additional 0.025 m padding
covers the bound on footprint displacement between the checked samples,
retaining the 0.15 m margin along the continuous geometric arc.

## Files and reproducible starts

`worlds/parking_lot.sdf` starts at the entrance. `case_s1.sdf` through
`case_s6.sdf` have identical obstacles and pedestrian scripts with a different
initial robot pose near the selected case. Reset simulation time and robot
state between trials, since the pedestrian phase affects the result.

`scene_ground_truth.json`, `parking_scenarios.json` and the reference CSVs
are independent evaluation material. Never substitute their exact goals,
object poses or expected decisions for your sensor-derived estimates.
