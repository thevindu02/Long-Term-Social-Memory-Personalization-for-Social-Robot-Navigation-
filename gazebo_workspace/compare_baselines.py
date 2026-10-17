#!/usr/bin/env python3
import json
import math
from comfort_signals.signals import project_point_to_polyline, compute_polyline_length

def compare_paths(file1, file2):
    with open(file1) as f:
        p1 = json.load(f)['agent1']
    with open(file2) as f:
        p2 = json.load(f)['agent1']

    l1 = compute_polyline_length(p1)
    l2 = compute_polyline_length(p2)
    diff_l = abs(l1 - l2)

    max_d12 = max(project_point_to_polyline(pt, p2)[1] for pt in p1)
    max_d21 = max(project_point_to_polyline(pt, p1)[1] for pt in p2)
    hausdorff = max(max_d12, max_d21)

    return {
        "len1": l1,
        "len2": l2,
        "len_diff": diff_l,
        "max_polyline_dist": hausdorff,
        "p1_samples": len(p1),
        "p2_samples": len(p2)
    }

if __name__ == '__main__':
    res = compare_paths(
        'evaluation_results/nominal_path_corridor_baseline_1.json',
        'evaluation_results/nominal_path_corridor_baseline_2.json'
    )
    print(json.dumps(res, indent=2))
