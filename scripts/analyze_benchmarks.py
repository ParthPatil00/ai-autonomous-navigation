"""Analyze post-fix benchmark results."""
import json
from pathlib import Path

with open('outputs/metrics/benchmark_results_postfix.json') as f:
    data = json.load(f)

meta = data['metadata']
records = data['records']

print('=== METADATA ===')
for k, v in meta.items():
    print(f'  {k}: {v}')

planners = ['A* (Euclidean)', 'A* (Octile)', 'Dijkstra']
print()
header = f"{'Planner':20s} | {'Success':8s} | {'Collisions':11s} | {'Mean PathLen':13s} | {'Mean Cost':10s} | {'Mean Nodes':11s} | {'Mean Plan ms':13s}"
print(header)
print('-' * len(header))

for name in planners:
    recs = [r for r in records if r['planner_name'] == name]
    plan_ok = sum(1 for r in recs if r['planning_succeeded'])
    goal_ok = sum(1 for r in recs if r['goal_reached'])
    collisions = sum(r['collision_count'] for r in recs)
    successes = [r for r in recs if r['planning_succeeded']]
    avg_len = sum(r['path_length'] for r in successes) / len(successes) if successes else 0
    avg_cost = sum(r['path_cost'] for r in successes) / len(successes) if successes else 0
    avg_nodes = sum(r['nodes_explored'] for r in recs) / len(recs) if recs else 0
    avg_plan = sum(r['planning_time_ms'] for r in recs) / len(recs) if recs else 0

    print(f'{name:20s} | {goal_ok:3d}/{len(recs):3d} | {collisions:11d} | {avg_len:13.2f} | {avg_cost:10.2f} | {avg_nodes:11.1f} | {avg_plan:13.2f}')

failures = [r for r in records if not r['goal_reached'] and r['planning_succeeded']]
no_path = [r for r in records if not r['planning_succeeded']]
coll_scenarios = [r for r in records if r['collision_count'] > 0]
print()
print(f'Total records: {len(records)}')
print(f'Planning failures (no path found): {len(no_path)}')
print(f'Goal not reached (but planned): {len(failures)}')
print(f'Scenarios with collisions: {len(coll_scenarios)}')

# Also check old baseline if it exists
old_path = Path('outputs/metrics/benchmark_results.json')
if old_path.exists():
    print('\n=== OLD BASELINE (pre-fix) ===')
    with open(old_path) as f:
        old_data = json.load(f)
    old_meta = old_data['metadata']
    old_records = old_data['records']
    print(f"  Version: {old_meta.get('benchmark_version', 'unknown')}")
    print(f"  base_seed: {old_meta.get('base_seed')}, scenarios: {old_meta.get('num_scenarios')}")
    for name in planners:
        recs = [r for r in old_records if r['planner_name'] == name]
        if not recs:
            continue
        goal_ok = sum(1 for r in recs if r['goal_reached'])
        collisions = sum(r['collision_count'] for r in recs)
        successes = [r for r in recs if r['planning_succeeded']]
        avg_cost = sum(r['path_cost'] for r in successes) / len(successes) if successes else 0
        avg_nodes = sum(r['nodes_explored'] for r in recs) / len(recs)
        avg_plan = sum(r['planning_time_ms'] for r in recs) / len(recs)
        print(f"  {name:20s} | {goal_ok:3d}/{len(recs):3d} | collisions={collisions} | avg_cost={avg_cost:.2f} | avg_nodes={avg_nodes:.1f} | avg_plan={avg_plan:.2f}ms")
else:
    print('\nNo old baseline found at outputs/metrics/benchmark_results.json')
