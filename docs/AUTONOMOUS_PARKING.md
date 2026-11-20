# Autonomous reverse parking

The source-tree entry point is `bash run_autonomous.sh` in ROS 2 Jazzy on
Ubuntu 24.04 with Gazebo Harmonic. The same mission runs in the static and
walking-actor worlds. The launch argument `scenario` selects a start pose.
All candidate poses come from measured scans. Evaluation ground truth has
its own topic and is used only in the recording and run evaluator.

## Default entrance demonstration

The entrance mode anchors the rounded rear-axle route in the SLAM frame at
startup. It follows the reference using continuous lookahead and a live laser
braking gate. If a section becomes blocked, a sensed free pose ahead on the
same section provides a Nav2 rejoin target. Fixed-case launches retain the
local survey and parking workflow.

The first circuit records the driven aisle and returns along the entrance
connector. At standstill it saves `first_lap_map.pgm` and starts the second
circuit. Configured distances along the course schedule inspections. Actual
free space, obstacles and candidate poses come from measured scans. The
white region is a pass-through. At each parking inspection, the node separates
moving returns, final-footprint checks, entry checks, and a complete Ackermann
reverse-path audit. A tracked obstacle at the mouth causes the inspection to
be skipped. The existing RGB camera is available for viewing.

## Algorithms

| Component | Implementation |
| --- | --- |
| Global localization | Online SLAM Toolbox, with the existing map-to-odometry transform and scan-matching configuration |
| Project implementation of the classroom algorithm | Inverse range model and incremental log odds in `parking_core.py`. Each scan removes duplicate evidence, gives hits priority over free rays, bounds evidence and retains unobserved cells |
| Local grid geometry | Scan-time transforms place returns in a metric odometry grid. Near-ground evidence is retained independently, so a ray above a curb cannot erase a low return |
| Course and recovery | Track the rounded course continuously with curve-speed regulation in the entrance mission. Plan a checked connection to a later point on the current section if needed. Fixed-case mode ranks local observed goals and retains short, audited recovery moves |
| Candidate discovery | Estimate dominant row directions from measured straight surfaces. Require sufficiently long neighboring side surfaces and comparable lane-facing ends, then score free footprint and boundary balance. Recheck remembered candidates against the latest evidence |
| Approach, reverse and comparison | Try analytically sampled reverse arcs and Nav2 Smac Hybrid A* with a Reeds–Shepp model. A rejected direct connection can prompt planning to a staging pose with a final reverse insertion. Audit interpolated full-wheel-envelope sweeps in the frozen evidence grid used for the final-pose baseline |
| Tracking and direction changes | Rear-axle Pure Pursuit with approach and curve speed reduction, stopping at each change of direction. Check final position, heading, free area and sustained standstill |
| Protective motion gate | The last publisher audits braking arcs using the measured speed, limited command, recent scans and a short forecast of tracked compact returns. Current, requested and straightening curvature are considered. A chassis contact latches a stop |

Version 0.5.1 excludes through aisles when both longitudinal extensions are
observed free or when the recorded survey trajectory crosses the candidate
from end to end. The same bay check runs again before declaring success.
Replaying the reported false stop by the white car rejected its candidate and
retained both previously verified S6 targets.

The self-implemented grid stays in continuous `odom`; the globally corrected
SLAM Toolbox map is displayed and used by Nav2. The mission freezes a local
evidence snapshot per planning request and rejects paths that fail that
snapshot's full-footprint audit. A transient guard stop can clear when the
space is observed clear again. Persistent blockage or departure from the
planned path triggers replanning from the current pose.

The drivetrain caps forward speed at 1.45 m/s. The entrance return
is limited to 0.85 m/s, the parking approach to 0.55 m/s, and the parking
reverse to 0.40 m/s. The live guard checks the measured speed, reaction
interval, braking distance, wheel envelope and tracked-obstacle forecast. The two extra short-range
scanners cover low objects in front and behind. Full-height objects are
sensed with the 360-degree scanner. Gazebo walking actors are sensed
obstacles without physical chassis contact response.

## Results

Each run creates one timestamped folder under `results/`.

- `result.json` records lap completion, ordered inspection outcomes and final estimated errors when parked.
- `decisions.jsonl` records paired final-pose and sweep outcomes, planner failures, and direction changes.
- `run.csv` records requested and final commands, steering, IMU, odometry, Gazebo rear-axle truth and sensor timestamps.
- `first_lap_map.pgm` and `first_lap_map.yaml` capture the map at the first return. `parking_map.pgm` and `parking_map.yaml` capture the final evidence map.

Use `python3 scripts/evaluate_run.py results/<run-directory>` to compute a
separate sampled static clearance and compare odometry with Gazebo pose.
The evaluator runs after the mission and has access to independent scene
geometry. Its polygon clearance is an estimate from sampled poses.
Add `--check-demonstration` to require the measured start return, the saved
first-lap map, the ordered inspection outcomes, and an entire vehicle
footprint inside the green reference zone with positive sampled clearance.

The current-model geometry check is `python3 scripts/validate_scenarios.py`.
The final-pose baseline and swept-path check share the same sensed candidate
and evidence snapshot. Path search exhaustion, occupied swept space and
insufficient observation are reported separately. Interpret outcomes per
attempt, including retries. The historical sedan-era case labels are design
references. Record fresh runs for LIMO.

## Verified two-lap entrance run

The default world ran with the red-bay pedestrian enabled in ROS 2 Jazzy
and Gazebo Harmonic. At 136.4 simulation seconds the first circuit ended
with a Gazebo-measured return error of 0.098 m and 0.23 degrees. The mission
saved the first-lap map and then inspected all five stations in order.

| Station | Measured outcome |
| --- | --- |
| White | Bypassed after observing the nearby obstacle |
| Blue | All nine mouth probes were blocked. Ten final poses fit, and all corresponding entry probes failed |
| Red | Nine tracking updates confirmed sustained motion in the bay mouth after standstill |
| Yellow | Fifteen final poses fit, and all corresponding entry probes failed |
| Green | Validated an Ackermann reverse trajectory, entered the space and confirmed standstill |

The mission finished at 283.3 simulation seconds with zero course rejoins
and zero reported chassis contacts. An independent evaluation placed the
entire vehicle in S1, the green bay. It measured 0.110 m position error,
0.124 degrees heading error against the parking-phase target, and 0.414 m
minimum sampled static clearance. Those numbers use Gazebo poses and the
scene geometry. The same evaluator records the accumulated drift from the
startup-only odometry alignment separately.

The [mission result](../results/verified_two_lap_entrance/result.json),
[decision log](../results/verified_two_lap_entrance/decisions.jsonl),
[first-lap map](../results/verified_two_lap_entrance/first_lap_map.yaml)
and [independent evaluation](../results/verified_two_lap_entrance/evaluation.json)
are included with the recorded trace.

## Historical static entrance run

A Gazebo Harmonic run on 19–20 November 2026 started at the overview entrance
with pedestrians disabled and completed at 185.7 seconds of simulation time.
It found the S6 area from scans and completed a perpendicular reverse
insertion. In that run, the final-pose baseline accepted seven attempts.
The sweep audit rejected six and accepted one. The independent evaluator
placed the entire final vehicle in S6, recorded no reported chassis contact,
and measured 0.110 m minimum sampled static clearance. The position and
heading errors in the parking-phase ground-truth alignment were 0.181 m and
4.26 degrees. The trace also includes transient braking holds.

The [raw mission result](../results/verified_static_entrance/result.json),
[decision log](../results/verified_static_entrance/decisions.jsonl), and
[independent evaluation](../results/verified_static_entrance/evaluation.json)
are included. Replay with the final candidate filter retained that S6
placement and rejected a sideways target found during an earlier trial
with walking actors. The later staging and analytical connection
alternatives can change which maneuver is chosen.

## Historical pedestrian run from the former S6 layout

With all six actors enabled, a run starting from the S6 case pose performed
survey, a tracking-error retreat, replanning and reverse entry. It passed
standstill verification at 81.4 simulation seconds. The independent
evaluation placed the final vehicle within S6, with 0.138 m position error,
2.03 degrees heading error and 0.040 m minimum sampled static clearance.
There was no reported chassis contact in this run. The [decision
log](../results/verified_pedestrian_s6/decisions.jsonl) and [independent
evaluation](../results/verified_pedestrian_s6/evaluation.json) are included.

The full entrance run with pedestrians did not complete validation. It was
able to stop and recover from some failed paths, then encountered a repeated
planning failure near the north side. An earlier trial also stopped with the
wrong orientation. Its candidate is rejected by the final geometric filter.
The actor-enabled S6 check does not measure minimum clearance to a person.

## Academic basis

- [Hess et al., Real-Time Loop Closure in 2D LIDAR SLAM](https://research.google.com/pubs/archive/45466.pdf)
- [Thrun, Learning Occupancy Grid Maps with Forward Sensor Models](https://doi.org/10.1023/A:1025584807625)
- [Jung et al., Scanning Laser Radar-Based Target Position Designation for Parking Aid System](https://doi.org/10.1109/TITS.2008.922980)
- [Dolgov et al., Path Planning for Autonomous Vehicles in Unknown Semi-Structured Environments](https://doi.org/10.1177/0278364909359210)
- [Reeds and Shepp, Optimal Paths for a Car That Goes Both Forwards and Backwards](https://msp.org/pjm/1990/145-2/pjm-v145-n2-p06-p.pdf)
- [Macenski et al., Regulated Pure Pursuit for Robot Path Tracking](https://doi.org/10.1007/s10514-023-10097-6)
- [Fox et al., The Dynamic Window Approach to Collision Avoidance](https://doi.org/10.1109/100.580977)
- [Wang et al., Hierarchical Planning for Autonomous Parking in Dynamic Environments](https://doi.org/10.1109/TCST.2024.3367468)
