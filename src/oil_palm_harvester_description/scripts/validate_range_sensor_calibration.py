#!/usr/bin/env python3
"""Validate a docking-range calibration configuration against the active URDF."""

import argparse
from pathlib import Path

from range_sensor_calibration_common import (
    fixed_transform_between,
    load_json,
    matrix_to_rpy,
    parse_fixed_joint_graph,
    require_simulation_config,
)


def default_share_directory():
    return Path(__file__).resolve().parent.parent


def main():
    share = default_share_directory()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', default=str(share / 'config' / 'range_sensor_calibration.nominal.json'))
    parser.add_argument(
        '--urdf', default=str(share / 'urdf' / 'oil_palm_harvester_kinematic.urdf'))
    parser.add_argument(
        '--mode', choices=('simulation', 'planning'), default='simulation')
    arguments = parser.parse_args()

    config = load_json(arguments.config)
    if arguments.mode == 'planning':
        if config.get('mode') != 'deployment_template':
            raise SystemExit('--mode planning accepts only a deployment_template configuration')
        print('Planning template is structurally present; unmeasured values remain intentionally null.')
        return

    require_simulation_config(config)
    links, graph = parse_fixed_joint_graph(arguments.urdf)
    reference = config['reference_frame']
    if reference not in links:
        raise SystemExit(f"URDF does not contain reference frame '{reference}'")

    print(f"Calibration ID: {config['calibration_id']}")
    print(f"Reference frame: {reference}")
    for name, sensor in config['sensors'].items():
        source = sensor['expected_frame_id']
        if source not in links:
            raise SystemExit(f"URDF does not contain sensor frame '{source}'")
        transform = fixed_transform_between(graph, reference, source)
        xyz = [transform[row][3] for row in range(3)]
        rpy = matrix_to_rpy(transform)
        print(
            f"T_{reference}_{source}: "
            f"xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) m, "
            f"rpy=({rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}) rad")
    print('PASS: all configured sensor frames are fixed relative to the docking reference.')


if __name__ == '__main__':
    main()
