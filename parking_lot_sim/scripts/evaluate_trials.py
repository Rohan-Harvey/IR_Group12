#!/usr/bin/env python3
from pathlib import Path
import argparse, json, sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from parking_system.simulation import run
from parking_system.evaluation import save_comparison, plot_run
if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--seeds', default='0,1,2,3,4')
    p.add_argument('--output', type=Path, default=ROOT / 'results')
    p.add_argument('--reuse', action='store_true')
    a = p.parse_args()
    reports = []
    for seed in [int(x) for x in a.seeds.split(',')]:
        directory = a.output / f'seed_{seed}'
        if a.reuse and (directory / 'report.json').is_file():
            report = json.loads((directory / 'report.json').read_text())
        else:
            report, *_ = run(ROOT, seed, directory)
        reports.append(report)
        print(json.dumps({k: report.get(k) for k in ['seed', 'status', 'laps', 'position_error_m', 'heading_error_deg', 'error']}), flush=True)
    result = save_comparison(ROOT, reports, a.output)
    plot_run(ROOT, a.output / f"seed_{reports[0]['seed']}")
    print(json.dumps({k: v for k, v in result.items() if k != 'decisions'}, indent=2))
    passed = all((r['status'] == 'COMPLETED' and (not r['contacts']) and (r['laps'] == 2) and (r.get('position_error_m', 99) < 0.3) and (r.get('heading_error_deg', 99) < 5) for r in reports))
    passed = passed and result['correctly_ordered_trials'] == len(reports) and (result['six_decision_trials'] == len(reports)) and (not result['proposed_unsafe_accepts']) and (not result['proposed_unnecessary_rejections'])
    raise SystemExit(0 if passed else 1)
