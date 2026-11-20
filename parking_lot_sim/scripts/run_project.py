#!/usr/bin/env python3
from pathlib import Path
import argparse, json, sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from parking_system.simulation import run
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--output', type=Path, default=ROOT / 'results/seed_0')
    p.add_argument('--observe-only', action='store_true')
    p.add_argument('--no-scan-match', action='store_true')
    p.add_argument('--initial-yaw-offset', type=float, default=0.0)
    a = p.parse_args()
    report, *_ = run(ROOT, a.seed, a.output, a.observe_only, not a.no_scan_match, a.initial_yaw_offset)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['status'] in ['COMPLETED', 'OBSERVED'] else 1)
