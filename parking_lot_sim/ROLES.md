# Six workstreams and implementation guide

This maps the six proposed responsibilities to the implementation. It is not an
assertion that the named members authored this generated code. The team must
review it, understand it, test it in Gazebo and record actual contributions.
The code and documentation in this release were substantially produced with
ChatGPT assistance; retain that fact in the separate AI statement.

| Proposed member | Workstream | Main files | Evidence |
| --- | --- | --- | --- |
| Abdullah Alqadi | Robot, sensors, environment and launch integration | `scripts/scene_robot.py`, `scripts/scene_people.py`, `scripts/scene_scenarios.py`, `scripts/generate_world.py`, `launch/`, `config/robot_bridge.yaml` | Dimension, asset, scene and route checks |
| Jenusan Ayathurai | Candidate generation and final-space assessment | `parking_system/perception.py` | Candidate records and frozen endpoint decisions in `results/decisions.csv` |
| Rohan Harvey | Reverse planning and swept-footprint validation | `parking_system/planning.py` | Analytic arcs, reverse-history connector, Hybrid A* fallback, continuous-motion subdivision and S5 enclosure test |
| Abdirahman Hirsi | Trials, independent reference cases and metrics | `parking_system/simulation.py`, `parking_system/evaluation.py`, `scripts/evaluate_trials.py`, `scripts/evaluate_live.py` | Seeded reports, comparison JSON/CSV, independent overlap checks and trajectory plot |
| Yu Chang | Localisation and taught occupancy mapping | `parking_system/mapping.py` | Encoder/IMU odometry, time pairing, scan registration, log-odds updates, map snapshots and estimate error |
| Yikun Yang | Tracking, state transitions and obstacle stopping | `parking_system/control.py`, `scripts/drive_project.py` | Two completed laps, guard stop/resume tests, stale-stream handling and stationary hold |

## Data flow

The scanner, rear encoders and IMU enter the runtime. Encoder/IMU integration
predicts motion. Local scan-to-map registration adjusts the pose, then the
student log-odds mapper accumulates free and occupied evidence. Small moving
clusters are tracked separately; persistent static surfaces suppress false
motion tracks. The map, recorded trajectory and unresolved motion regions feed
candidate assessment. A validated reverse path enters the tracker and live
laser guard. Runtime state and decisions go to the independent evaluator.

Only the simulator and evaluator read the generated world geometry or labelled
reference cases. Runtime mapping, assessment, planning and control do not read
case IDs, case goal poses, expected answers or simulator robot poses. The
runtime names opportunities P01, P02, etc. Evaluation associates those measured
opportunities with S1–S6 afterwards. The observation corridor and initial pose
are supplied mission parameters, as requested; this is not general autonomous
exploration of an arbitrary lot.

## Environment and units

The active LIMO retains its scaled mesh, collision, wheel and steering geometry.
Every parked vehicle has the same overall straight-wheel visual length, width
and height. Parked vehicles use a coloured body and a shared 176-triangle wheel
mesh, keeping rendering light. Static vehicle collisions bound those visuals.
The active planner uses the larger full steered-wheel envelope, including
body overhang, rather than the narrower parked-wheel footprint.

All computation uses metres and radians. The known initial pose anchors the
map frame; the ROS adapter publishes map → odom → base_footprint. Fixed robot
link transforms come from the URDF. The sensor is centred at the declared
height, with robot visuals excluded from its GPU scan. Boundary barriers are
visible at that height. Animated people have no Gazebo physical contact
response, so live stopping uses their laser returns.

## Estimation and the taught method

`EncoderImuOdometry` integrates the mean rear-wheel displacement at the rear
axle and uses unwrapped IMU yaw. `PoseBuffer` pairs scans with interpolated
encoder poses at the scan timestamp. `LocalSlam` predicts relative motion,
registers returns to the accumulated map using bounded point-to-point
least-squares corrections, and inserts the corrected scan.

The Gazebo IMU reference is the initial sensor orientation, so the first received
yaw is retained rather than re-zeroed. The default simulator configuration holds
scan matching to that heading and estimates translation from point residuals.
This prevents incremental scan registration from accumulating a conflicting yaw
reference. The general three-variable registration path remains configurable.
A startup delay lets the physical model settle before driving.

The occupancy update implements the taught binary Bayes/log-odds method:
free evidence is logit(0.3), hit evidence is logit(0.8), the prior is 0.5, and
values are clamped. A separate observed flag preserves unknown cells. No-hit
rays clear up to the range limit and do not create occupied endpoints. Space
behind an occluding hit remains unknown. Unit tests verify these semantics.

This is local incremental scan-to-map SLAM. It has no global pose-graph
optimisation or loop-closure correction of old scans. Past inserted poses remain
fixed. Repeated local registration is not described as global loop closure.
Error values come from aligned initial-frame simulator evaluation, and scan
registration is not guaranteed to improve on odometry in every tested run.

## Detection and assessment

The detector searches the declared outer parking strips for observed free
intervals bounded by obstacle evidence. It groups intervals across depth,
requires sustained depth support, and orders the resulting opportunities by
the observation route. It does not load the six labelled test regions.
Endpoint options use measured gap location and depth, with a perpendicular
heading. The full oriented footprint plus the configured uncertainty/tracking
margin must lie in observed free cells. Unknown cells fail the check.

The endpoint-only baseline is frozen before manoeuvre and dynamic checks.
The proposed method also requires a validated steering-feasible reverse
motion. Unresolved moving occupancy in the bay entry is recorded as a
temporary rejection requiring fresh reassessment. This dynamic guard is
reported separately from the static manoeuvre comparison; it is not evidence
that a statically usable space is geometrically impossible.

## Planning and the independent negative case

The planner tries quarter-circle reverse entries on both sides and connects
them to the current pose by retracing a recently observed, driven corridor in
reverse. It refines that guide with a curvature-limited bicycle rollout and
checks the complete swept body. This allows the robot to finish both full laps,
then reverse back along the empty west aisle to S6 without driving a third lap
past the rejected cases. A reverse-only Hybrid A* primitive search is the
bounded fallback. Forward shunting is outside this release's parking scope.

Swept checking subdivides by translation plus body-corner rotation, limiting
maximum corner displacement per check. It checks the interior and boundary of
the oriented footprint against occupied and unknown cells. The parking margin
covers discretisation and a bounded estimate/tracking allowance. It is an
engineering bound, not a formally calibrated probability of collision.

A search failure is reported as a finite search outcome, not a proof of
physical impossibility. S5 has a separate geometric reference: the chassis
contains an inscribed disc. The side vehicles, approach blockers and back
barrier form a closed chain whose gaps are smaller than that disc's diameter.
The endpoint fits inside that enclosure, but the robot cannot cross the chain.
The enclosure test samples the chain with an additional Lipschitz allowance.

## Control, safety and completion

The supervisor moves through OBSERVE, SETTLE, PARK, HOLD and COMPLETE, or fails
with a recorded reason. Exactly two closed CCW laps precede assessment. Pure
pursuit follows rear-axle targets with signed reverse speed and bounded
curvature. The live scan guard tests a predicted stopping region in forward
and reverse, holds zero command when blocked and resumes after clearance.
A persistent obstacle, stale/invalid stream, clock reset, large deviation or
lack of measured motion aborts the mission. Completion requires a stationary
hold within the declared estimated-pose tolerances.

The independent offline evaluator checks geometric overlap each 50 ms, records
minimum clearance on its sampling schedule, and measures achieved position and
yaw against the selected measured target. The ROS evaluator reads simulator
poses separately and labels geometric overlap. It does not feed back into the
controller. Physical Gazebo contact forces are not measured by these reports.

## What has and has not been validated

Run `python3 -m unittest discover -s tests -v` and
`python3 scripts/evaluate_trials.py --seeds 0,1,2,3,4` from the package.
`results/comparison.json` contains the actual trial counts and errors; do not
replace them with assumed values. Noise enters the laser and encoder/IMU
measurements before mapping and control. The deterministic 2D ray caster is a
sensor test environment, and the vehicle dynamics include acceleration and
steering slew. It is not Gazebo physics.

This workspace does not contain ROS, Gazebo, colcon or CMake. The ROS adapter
and launch files have source/contract checks, but have not been built or run
against those installed systems here. Before claiming a successful live demo,
measure wheel slip, steering arcs, actual topic rates, scan visibility,
Gazebo real-time factor, final pose, stopping and physical contacts. Preserve
any failed runs and update the report with measured outcomes.
