# Ultra-light revision

Use the main README for current instructions. This revision is based on the
attached `Parking_Lot_Simulation (3).zip`, not a different teammate version.

- All CAD mesh references and actors are removed from every generated world.
- No animation rig, camera, camera bridge, rear scanner or contact sensor runs.
- Robot body/wheel visuals are primitives; original scaled drive geometry,
  chassis/wheel collision geometry and inertia parameters remain.
- Every parked car uses its original envelope as one visible/colliding block.
- Trees, lamps, grass/shrubs, decorative pavement and extra GUI panels are removed.
- One centre-mounted 360° GPU scan replaces two half scans. Robot visual flags
  exclude self-returns; all scenery remains visible to the laser.
- IMU/encoder streams needed by the existing estimator remain at 20 Hz.
- Joint publication is explicitly throttled to 20 Hz; physics is 200 Hz.
- The optional bare world removes all sensors and sensor-system plugins plus
  joint/evaluation publishers; automatic driving is blocked in this mode.

S2 has no moving blocker and is suitable in the temporary static configuration.
The preset demo still targets S6. All other case geometry and rejection tests
are retained. The removed contact sensor cannot certify collision-free driving;
run reports explicitly mark it unavailable instead of claiming it passed.

The latest measurements and offline results are in `config/performance_report.json`
and `docs/validation_summary.json`. These describe package geometry/configuration
and Python tests, not live Gazebo performance. Source Xacro files are retained
for provenance; no heavy vendor meshes are included or loaded.
