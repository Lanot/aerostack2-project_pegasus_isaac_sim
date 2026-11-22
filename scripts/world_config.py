"""Read the shared Gazebo-style world schema without importing Isaac Sim or ROS."""
import argparse
import copy
import json
import math
from pathlib import Path
import re
import warnings

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]


def vector(value, name, size=3):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must contain {size} numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name} must contain finite numbers')
    return [float(v) for v in value]


def load_world(filename):
    path = Path(filename).expanduser().resolve()
    with path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict) or not isinstance(config.get('drones'), list) or not config['drones']:
        raise ValueError('World config must contain a nonempty drones list')
    result = copy.deepcopy(config)
    result['config_file'] = str(path)
    result['world_name'] = config.get('world_name', 'empty')
    if not isinstance(result['world_name'], str):
        raise ValueError('world_name must be a string')
    for key, default in (('ros2', True), ('follow_camera', False)):
        result.setdefault(key, default)
        if not isinstance(result[key], bool):
            raise ValueError(f'{key} must be a boolean')
    if 'environment' in result and not isinstance(result['environment'], str):
        raise ValueError('environment must be a string')
    px4 = result.setdefault('px4', {})
    if not isinstance(px4, dict):
        raise ValueError('px4 must be a mapping')
    for key in ('enabled', 'autolaunch'):
        px4.setdefault(key, True)
        if not isinstance(px4[key], bool):
            raise ValueError(f'px4.{key} must be a boolean')
    px4.setdefault('model', 'gazebo-classic_iris')
    if not isinstance(px4['model'], str) or not px4['model']:
        raise ValueError('px4.model must be a nonempty string')
    if 'origin' in config:
        origin = config['origin']
        if not isinstance(origin, dict):
            raise ValueError('origin must contain latitude, longitude and altitude')
        values = vector([origin.get(k) for k in ('latitude', 'longitude', 'altitude')], 'origin')
        if not -90 <= values[0] <= 90 or not -180 <= values[1] <= 180:
            raise ValueError('Invalid geographic origin')
        result['origin'] = dict(zip(('latitude', 'longitude', 'altitude'), values))
    if config.get('world_usd'):
        usd = config['world_usd']
        if not isinstance(usd, str):
            raise ValueError('world_usd must be a string')
        if '://' not in usd:
            usd_path = Path(usd).expanduser()
            if not usd_path.is_absolute():
                usd_path = path.parent / usd_path
            usd = str(usd_path.resolve())
            if not Path(usd).is_file():
                raise ValueError(f'World USD not found: {usd}')
        result['world_usd'] = usd
    if 'scene_camera_view' in result:
        view = result['scene_camera_view']
        if not isinstance(view, dict):
            raise ValueError('scene_camera_view must contain eye and target')
        view['eye'] = vector(view.get('eye'), 'scene_camera_view.eye')
        view['target'] = vector(view.get('target'), 'scene_camera_view.target')
        if view['eye'] == view['target']:
            raise ValueError('Camera eye and target must differ')
    names, instances = set(), set()
    for index, drone in enumerate(result['drones']):
        if not isinstance(drone, dict):
            raise ValueError(f'drones[{index}] must be a mapping')
        name = drone.get('model_name')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name):
            raise ValueError('Each drone needs a valid ROS/USD model_name')
        if name in names:
            raise ValueError(f'Duplicate drone name: {name}')
        instance = drone.get('instance', index)
        if isinstance(instance, bool) or not isinstance(instance, int) or not 0 <= instance <= 254:
            raise ValueError(f'{name}.instance must be an integer in 0..254')
        if instance in instances:
            raise ValueError(f'Duplicate PX4 instance: {instance}')
        names.add(name)
        instances.add(instance)
        drone['instance'] = instance
        drone['xyz'] = vector(drone.get('xyz', [0, 0, 0.07]), name + '.xyz')
        drone['rpy'] = vector(drone.get('rpy', [0, 0, 0]), name + '.rpy')
        drone.setdefault('model_type', 'quadrotor_base')
        if not isinstance(drone['model_type'], str):
            raise ValueError('model_type must be a string')
        if not isinstance(drone.get('payload', []), list):
            raise ValueError(f'{name}.payload must be a list')
        if drone.get('usd_path'):
            usd = Path(drone['usd_path']).expanduser()
            if not usd.is_absolute():
                usd = path.parent / usd
            if not usd.is_file():
                raise ValueError(f'Drone USD not found: {usd}')
            drone['usd_path'] = str(usd.resolve())
    if 'follow_drone' in result and (not isinstance(result['follow_drone'], str) or result['follow_drone'] not in names):
        raise ValueError('follow_drone must name a configured drone')
    return result


def payloads(drone):
    """Flatten nested Gazebo gimbal payloads, retaining their fixed sensors."""
    def visit(items):
        for item in items:
            if not isinstance(item, dict):
                raise ValueError('Each payload must be a mapping')
            kind = item.get('model_type')
            if not isinstance(kind, str):
                raise ValueError('Each payload needs model_type')
            if kind in ('gimbal_speed', 'gimbal_position'):
                warnings.warn(f'{drone["model_name"]}: {kind} is a fixed camera mount in Pegasus; no gimbal actuation')
                child = item.get('payload', [])
                yield from visit([child] if isinstance(child, dict) else child)
            elif kind in ('gps', 'hd_camera', 'camera', 'monocular_camera', 'lidar', 'lidar3d', 'gpu_lidar'):
                yield item
            else:
                raise ValueError(f'Unsupported Pegasus payload model_type: {kind}')
    return list(visit(drone.get('payload', [])))


def platform_config(world, name, base_file):
    """Generate exact per-drone routing; never reuse stale numbered ID mappings."""
    drone = next((d for d in world['drones'] if d['model_name'] == name), None)
    if drone is None:
        raise ValueError(f'Drone {name!r} is not in the world config')
    with Path(base_file).open() as stream:
        base = yaml.safe_load(stream) or {}
    common = base.get('/**', {})
    params = dict(common.get('ros__parameters', {}))
    params.update(common.get('platform', {}).get('ros__parameters', {}))
    params.update(base.get(name, {}).get('platform', {}).get('ros__parameters', {}))
    params.update(fmu_prefix='/' + name, target_system_id=drone['instance'] + 1, use_sim_time=True)
    return {'/**': {'ros__parameters': params}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--names', action='store_true')
    parser.add_argument('--platform-config')
    parser.add_argument('--base-config', default=str(PROJECT_ROOT / 'config/config.yaml'))
    parser.add_argument('--output')
    args = parser.parse_args()
    try:
        world = load_world(args.config)
        if args.platform_config:
            if not args.output:
                parser.error('--output is required with --platform-config')
            Path(args.output).write_text(yaml.safe_dump(platform_config(world, args.platform_config, args.base_config)))
        elif args.names:
            print(','.join(d['model_name'] for d in world['drones']))
        else:
            print(json.dumps(world, indent=2))
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
