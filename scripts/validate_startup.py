#!/usr/bin/env python3
from pathlib import Path
import argparse, json, sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from parking_system.simulation import run, SensorWorld
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, default=ROOT / 'results/startup')
    p.add_argument('--range', type=float, default=20.0)
    args = p.parse_args()
    original = SensorWorld.scan

    def limited(self, pose, now, angles, rng, noise=0.01, maximum=35):
        return original(self, pose, now, angles, rng, noise, args.range)
    SensorWorld.scan = limited
    reports = []
    try:
        for offset in [-0.1, 0.1]:
            report, *_ = run(ROOT, 0, args.output / ('negative' if offset < 0 else 'positive'), observe_only=True, initial_yaw_offset=offset)
            reports.append({k: report.get(k) for k in ['kind', 'status', 'error', 'initial_yaw_offset_rad', 'laps', 'contacts', 'final_true_pose', 'final_estimated_pose']})
            print(json.dumps(reports[-1]), flush=True)
    finally:
        SensorWorld.scan = original
    summary = {'runtime_version': '1.0.1', 'sensor_range_m': args.range, 'scope': 'Offline startup settling-rotation regression; observation phase only, not Gazebo.', 'cases': reports, 'passed': all((r['status'] == 'OBSERVED' and r['laps'] == 2 and (not r['contacts']) for r in reports))}
    (ROOT / 'results/startup_regression.json').write_text(json.dumps(summary, indent=2) + '\n')
    raise SystemExit(0 if summary['passed'] else 1)
