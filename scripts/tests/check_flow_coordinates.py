#!/usr/bin/env python3
"""Validate observed flow coordinates separately from legacy geometry errors."""
import argparse
import collections
import csv
import json
import math
from pathlib import Path


def check(run):
    with (run / 'data/field_diagnostics.csv').open() as f:
        rows = list(csv.DictReader(f))
    active = [r for r in rows if math.hypot(float(r['intent_vx']), float(r['intent_vy'])) > 0.1]
    valid = [r for r in active if float(r['flow_coordinate_valid']) == 1.0]
    assert valid, 'No valid flow coordinates during active navigation'
    for r in valid:
        eta = [float(r[k]) for k in ('flow_eta_0', 'flow_eta_1', 'flow_eta_norm')]
        assert all(math.isfinite(x) for x in eta)
        assert abs(math.hypot(*eta[:2]) - eta[2]) < 3e-8, 'eta norm mismatch'
        assert float(r['flow_section_residual']) <= 1.1e-8, 'section residual'
    for previous, current in zip(active, active[1:]):
        if (current['flow_field_version'] == previous['flow_field_version'] and
                float(current['flow_field_version']) > 0):
            assert all(current[k] == previous[k] for k in
                       ('flow_anchor_x', 'flow_anchor_y', 'flow_anchor_z')), 'anchor moved within interval'
    for r in active:
        if float(r['flow_coordinate_valid']) != 1.0:
            assert r['flow_eta_norm'] == '', 'invalid coordinate presented as an error'
    with (run / 'data/actual_trajectory.csv').open() as f:
        actual = list(csv.DictReader(f))
    result = {
        'run': str(run), 'active_rows': len(active), 'valid_rows': len(valid),
        'valid_fraction': len(valid)/len(active),
        'status_counts': dict(collections.Counter(r['flow_coordinate_status'] for r in active)),
        'max_section_residual': max(float(r['flow_section_residual']) for r in valid),
        'max_eta_norm': max(float(r['flow_eta_norm']) for r in valid),
        'max_compute_ms': max(float(r['flow_compute_ms']) for r in active),
        'mean_compute_ms': sum(float(r['flow_compute_ms']) for r in active)/len(active),
        'max_gradient_relative_error': max(float(r['flow_gradient_relative_error']) for r in valid),
        'start_y': float(actual[0]['y']), 'end_y': float(actual[-1]['y']),
    }
    metadata = dict(line.split('=', 1) for line in (run / 'metadata.txt').read_text().splitlines()
                    if '=' in line)
    if Path(metadata.get('trace_file', '')).name == 'pillar_forest_crossing_v1.yaml':
        assert result['start_y'] > 13.0 and result['end_y'] < -13.5, 'did not cross the arena'
    if active and 'flow_control_valid' in active[0]:
        controlled = [r for r in active if float(r['flow_control_valid']) == 1.0]
        assert controlled, 'No valid nominal controls'
        for r in active:
            nominal = [float(r['flow_nominal_v'+axis]) for axis in ('x', 'y', 'z')]
            final = [float(r['flow_final_v'+axis]) for axis in ('x', 'y', 'z')]
            delta = [float(r['flow_delta_v'+axis]) for axis in ('x', 'y', 'z')]
            assert all(abs(f-n-d)<3e-8 for f,n,d in zip(final,nominal,delta))
            if float(r['flow_control_valid']) != 1.0:
                assert all(v == 0.0 for v in nominal), 'invalid nominal command was nonzero'
        for r in controlled:
            assert float(r['flow_JJdag_residual']) <= 1e-8
            assert float(r['flow_j_sigma_min']) >= 1e-4
            assert float(r['flow_j_condition']) <= 1e3
        result.update({
            'control_valid_rows': len(controlled),
            'control_status_counts': dict(collections.Counter(r['flow_control_status'] for r in active)),
            'max_Ju': max(float(r['flow_Ju_residual']) for r in controlled),
            'max_JJdag_residual': max(float(r['flow_JJdag_residual']) for r in controlled),
            'max_nominal_decay_residual': max(float(r['flow_decay_residual']) for r in controlled),
            'max_control_compute_ms': max(float(r['flow_control_compute_ms']) for r in active),
        })
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    check(p.parse_args().run)
