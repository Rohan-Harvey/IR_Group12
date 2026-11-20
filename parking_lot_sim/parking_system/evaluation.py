from pathlib import Path
import csv, json, math
import numpy as np
from .geometry import rectangle

def match_decisions(report, manifest):
    cases = manifest['cases']
    records = []
    used = set()
    for decision in report['decisions']:
        point = decision['detected_pose'][:2]
        distances = [math.dist(point, c['goal_pose_m_deg'][:2]) if c['id'] not in used else math.inf for c in cases]
        index = int(np.argmin(distances))
        case = cases[index] if distances[index] < 3.0 else None
        if case:
            used.add(case['id'])
        row = {k: v for k, v in decision.items() if not k.startswith('_')}
        row['reference_case'] = case['id'] if case else 'UNMATCHED'
        row['reference_expected'] = case['expected'] if case else 'UNRESOLVED'
        row['correct_decision'] = case is not None and (decision['decision'] == 'ACCEPT') == (case['expected'] == 'ACCEPT')
        records.append(row)
    return records

def summary(reports, manifest):
    all_rows = []
    for report in reports:
        for row in match_decisions(report, manifest):
            all_rows.append({'seed': report['seed'], **row})
    unsafe = [r for r in all_rows if r['reference_expected'] not in ['ACCEPT', 'UNRESOLVED']]
    positive = [r for r in all_rows if r['reference_expected'] == 'ACCEPT']
    static = [r for r in unsafe if r['reference_expected'] == 'REJECT']
    dynamic = [r for r in unsafe if r['reference_expected'] == 'REJECT_DYNAMIC']
    return {'kind': 'Offline sensor/bicycle simulation only; not a live Gazebo or real-robot validation', 'trials': len(reports), 'seeds': [r['seed'] for r in reports], 'completed_trials': sum((r['status'] == 'COMPLETED' for r in reports)), 'collision_trials': sum((bool(r['contacts']) for r in reports)), 'two_lap_trials': sum((r['laps'] == 2 for r in reports)), 'six_decision_trials': sum((len(r['decisions']) == 6 for r in reports)), 'correctly_ordered_trials': sum(([r['reference_case'] for r in match_decisions(report, manifest)] == ['S1', 'S2', 'S3', 'S4', 'S5', 'S6'] for report in reports)), 'endpoint_baseline_unsafe_accepts': sum((r['endpoint_baseline_accept'] for r in unsafe)), 'unsafe_reference_decisions': len(unsafe), 'static_rejection_reference_decisions': len(static), 'endpoint_baseline_static_false_accepts': sum((r['endpoint_baseline_accept'] for r in static)), 'manoeuvre_checker_static_false_accepts': sum((r['manoeuvre_accept'] for r in static)), 'dynamic_rejection_reference_decisions': len(dynamic), 'dynamic_endpoint_baseline_accepts': sum((r['endpoint_baseline_accept'] for r in dynamic)), 'dynamic_guard_rejections': sum((not r['dynamic_guard_clear'] for r in dynamic)), 'proposed_unsafe_accepts': sum((r['decision'] == 'ACCEPT' for r in unsafe)), 'proposed_unnecessary_rejections': sum((r['decision'] != 'ACCEPT' for r in positive)), 'positive_reference_decisions': len(positive), 'position_error_m_max': max((r.get('position_error_m', math.inf) for r in reports), default=None), 'heading_error_deg_max': max((r.get('heading_error_deg', math.inf) for r in reports), default=None), 'minimum_clearance_m': min((r['minimum_true_clearance_m'] for r in reports), default=None), 'scan_processing_ms_mean': float(np.mean([r['scan_processing_mean_ms'] for r in reports])) if reports else None, 'notes': ['The endpoint baseline is frozen before manoeuvre and dynamic checks on the same map and candidate.', 'Dynamic rejection is recorded separately from geometric manoeuvre rejection.', 'A search-budget failure is not an independent proof of physical impossibility.', 'Geometric overlap is checked every 50 ms; physical Gazebo contacts are not measured.'], 'decisions': all_rows}

def save_comparison(root, reports, directory):
    root = Path(root)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    manifest = json.loads((root / 'config/parking_scenarios.json').read_text())
    result = summary(reports, manifest)
    (directory / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    fields = ['seed', 'reference_case', 'candidate_id', 'reference_expected', 'endpoint_baseline_accept', 'manoeuvre_accept', 'dynamic_guard_clear', 'decision', 'reason', 'planning_status', 'observed_free_fraction', 'assessment_ms']
    with (directory / 'decisions.csv').open('w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(result['decisions'])
    return result

def plot_run(root, directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from .simulation import load_world
    root = Path(root)
    directory = Path(directory)
    report = json.loads((directory / 'report.json').read_text())
    data = np.load(directory / 'trace.npz')
    grid = np.load(directory / ('assessment_map.npz' if (directory / 'assessment_map.npz').is_file() else 'estimated_map.npz'))
    manifest = json.loads((root / 'config/parking_scenarios.json').read_text())
    cfg = json.loads((root / 'config/autonomy.json').read_text())
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.5), gridspec_kw={'width_ratios': [1.3, 1]})
    ax, second = axes
    for name, poly in load_world(root).polygons:
        ax.add_patch(Polygon(poly, facecolor='#d1d5db', edgecolor='#6b7280', lw=0.45))
    samples = data['samples']
    ax.plot(samples[:, 1], samples[:, 2], color='#1d4ed8', lw=1.7, label='Actual simulated trajectory')
    ax.plot(samples[:, 4], samples[:, 5], color='#f97316', lw=0.75, alpha=0.8, label='Estimated trajectory')
    reverse = data['reverse']
    if len(reverse):
        ax.plot(reverse[:, 0], reverse[:, 1], color='#16a34a', ls='--', lw=2, label='Validated reverse plan')
    for case in manifest['cases']:
        x, y, h = case['goal_pose_m_deg']
        color = '#15803d' if case['id'] == 'S6' else '#b91c1c'
        ax.text(x, y, case['id'], ha='center', va='center', color=color, fontsize=11, fontweight='bold', bbox={'facecolor': 'white', 'alpha': 0.9, 'edgecolor': 'none', 'pad': 2})
    ax.add_patch(Polygon(rectangle(report['final_true_pose'], cfg['robot']['length'], cfg['robot']['width']), facecolor='#22c55e', alpha=0.8))
    ax.annotate('Entrance', xy=cfg['start_pose'][:2], xytext=(-39, -16), arrowprops={'arrowstyle': '->'}, fontsize=10)
    ax.set(xlim=(-42, 29), ylim=(-22, 22), xlabel='x (m)', ylabel='y (m)', title='Two CCW laps, then reverse into the last bay')
    ax.set_aspect('equal')
    ax.legend(loc='lower right', fontsize=8)
    probability = 1 / (1 + np.exp(-grid['logodds']))
    image = np.where(grid['observed'], probability, 0.5)
    origin = grid['origin']
    resolution = float(grid['resolution'])
    height, width = image.shape
    second.imshow(image, origin='lower', extent=[origin[0], origin[0] + width * resolution, origin[1], origin[1] + height * resolution], cmap='gray_r', vmin=0, vmax=1)
    for row in match_decisions(report, manifest):
        x, y, h = row['detected_pose']
        second.text(x, y, row['reference_case'], ha='center', color='#15803d' if row['decision'] == 'ACCEPT' else '#b91c1c', fontsize=10, fontweight='bold')
    second.set(xlim=(-33, 27), ylim=(-20, 20), xlabel='x (m)', ylabel='y (m)', title='Estimated occupancy map used for assessment')
    second.set_aspect('equal')
    subtitle = f"Offline seed {report['seed']} | {report['status']} | {report['laps']} laps | {len(report['contacts'])} geometric overlaps | final error {report.get('position_error_m', 0):.3f} m, {report.get('heading_error_deg', 0):.2f}°"
    fig.suptitle(subtitle, fontsize=11)
    fig.tight_layout()
    path = directory / 'run_overview.png'
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path
