"""Read the shared Gazebo-style world schema without importing Isaac Sim or ROS."""
import argparse
import copy
import json
import math
from pathlib import Path
import re
import warnings

import yaml

from sensor_rates import validate_sensor_rates

PROJECT_ROOT = Path(__file__).resolve().parents[1]

SIMULATION_DEFAULTS = {
    'physics_device': 'cuda', 'physics_hz': 250, 'render_hz': 50,
    'physics_threads': -1, 'profile': False, 'render_quality': 'current',
    'direct_tensors': False,
}


def add_simulation_arguments(parser):
    """Shared CLI overrides; unspecified values retain the world YAML setting."""
    parser.add_argument('--physics-device', '--physics_device', choices=('cpu', 'cuda'))
    parser.add_argument('--physics-hz', '--physics_hz', type=int)
    parser.add_argument('--physics-threads', '--physics_threads', type=int)
    parser.add_argument('--profile', action=argparse.BooleanOptionalAction, default=None)
    parser.add_argument('--render-quality', '--render_quality', choices=('current', 'fast'))
    parser.add_argument('--direct-tensors', '--direct_tensors',
                        action=argparse.BooleanOptionalAction, default=None)


def simulation_overrides(args):
    return {key: getattr(args, key) for key in SIMULATION_DEFAULTS
            if getattr(args, key, None) is not None}


def vector(value, name, size=3):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f'{name} must contain {size} numbers')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name} must contain finite numbers')
    return [float(v) for v in value]


def load_world(filename, overrides=None):
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

    for key, default in (('ros2', True), ('ros2_graphical_sensors', True), ('follow_camera', False)):
        result.setdefault(key, default)
        if not isinstance(result[key], bool):
            raise ValueError(f'{key} must be a boolean')

    if 'environment' in result and not isinstance(result['environment'], str):
        raise ValueError('environment must be a string')

    px4 = result.setdefault('px4', {})
    if not isinstance(px4, dict):
        raise ValueError('px4 must be a mapping')
    for key in ('enabled', 'autolaunch', 'enable_lockstep'):
        px4.setdefault(key, True)
        if not isinstance(px4[key], bool):
            raise ValueError(f'px4.{key} must be a boolean')
    px4.setdefault('model', 'gazebo-classic_iris')
    if not isinstance(px4['model'], str) or not px4['model']:
        raise ValueError('px4.model must be a nonempty string')

    simulation = result.setdefault('simulation', {})
    if not isinstance(simulation, dict):
        raise ValueError('simulation must be a mapping')
    if overrides:
        unknown = set(overrides) - SIMULATION_DEFAULTS.keys()
        if unknown:
            raise ValueError(f'Unknown simulation overrides: {sorted(unknown)}')
        simulation.update(overrides)
    for key, default in SIMULATION_DEFAULTS.items():
        simulation.setdefault(key, default)
    if simulation['physics_device'] not in ('cpu', 'cuda'):
        raise ValueError('simulation.physics_device must be cpu or cuda')
    if simulation['render_quality'] not in ('current', 'fast'):
        raise ValueError('simulation.render_quality must be current or fast')
    threads = simulation['physics_threads']
    if isinstance(threads, bool) or not isinstance(threads, int) or threads < -1:
        raise ValueError('simulation.physics_threads must be -1 or a nonnegative integer')
    if not isinstance(simulation['direct_tensors'], bool):
        raise ValueError('simulation.direct_tensors must be a boolean')
    for key, default in (('physics_hz', 250), ('render_hz', 50)):
        simulation.setdefault(key, default)
        value = simulation[key]
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            raise ValueError(f'simulation.{key} must be a positive integer')
    simulation.setdefault('profile', False)
    if not isinstance(simulation['profile'], bool):
        raise ValueError('simulation.profile must be a boolean')
    if simulation['render_hz'] > simulation['physics_hz']:
        raise ValueError('simulation.render_hz must not exceed physics_hz')

    validate_sensor_rates(simulation.get('sensor_rates', {}), simulation['physics_hz'])

    extensions = result.setdefault('extensions', [])
    if not isinstance(extensions, list) or any(
        not isinstance(name, str) or not name or name.startswith('-') for name in extensions
    ):
        raise ValueError('extensions must be a list of extension names')

    extension_paths = result.setdefault('extension_paths', [])
    if not isinstance(extension_paths, list):
        raise ValueError('extension_paths must be a list of local directories')
    normalized_paths = []

    for entry in extension_paths:
        if not isinstance(entry, str) or not entry:
            raise ValueError('extension_paths must contain nonempty strings')

        directory = Path(entry).expanduser()
        if not directory.is_absolute():
            directory = path.parent / directory
        directory = directory.resolve()
        if not directory.is_dir():
            raise ValueError(f'Extension directory not found: {directory}')
        normalized_paths.append(str(directory))

    result['extension_paths'] = normalized_paths

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

    # Validate camera timing before Kit starts, including nested gimbal payloads.
    for drone in result['drones']:
        for sensor in payloads(drone, warn_gimbals=False):
            if sensor['model_type'] in ('hd_camera', 'camera', 'monocular_camera'):
                camera_frequency(sensor, result['simulation']['render_hz'], drone['model_name'])
    return result


def payloads(drone, *, warn_gimbals=True):
    """Flatten nested Gazebo gimbal payloads, retaining their fixed sensors."""
    def visit(items):
        for item in items:
            if not isinstance(item, dict):
                raise ValueError('Each payload must be a mapping')
            kind = item.get('model_type')
            if not isinstance(kind, str):
                raise ValueError('Each payload needs model_type')
            if kind in ('gimbal_speed', 'gimbal_position'):
                if warn_gimbals:
                    warnings.warn(f'{drone["model_name"]}: {kind} is a fixed camera mount in Pegasus; no gimbal actuation')
                child = item.get('payload', [])
                yield from visit([child] if isinstance(child, dict) else child)
            elif kind in ('gps', 'hd_camera', 'camera', 'monocular_camera', 'lidar', 'lidar3d', 'gpu_lidar'):
                yield item
            else:
                raise ValueError(f'Unsupported Pegasus payload model_type: {kind}')

    return list(visit(drone.get('payload', [])))


def camera_frequency(sensor, render_hz, drone_name):
    """Choose a valid default, or reject an incompatible explicit camera rate."""
    settings = sensor.get('config', {})
    name = f"{drone_name}/{sensor.get('model_name', 'camera')}"
    if not isinstance(settings, dict):
        raise ValueError(f'{name}: camera config must be a mapping')
    # Highest divisor up to 30 Hz: 25 Hz at 50 Hz rendering, 30 Hz at 60 Hz.
    default = max(rate for rate in range(1, min(30, render_hz) + 1) if render_hz % rate == 0)
    rate = settings.get('frequency', default)
    if (isinstance(rate, bool) or not isinstance(rate, (int, float))
            or not math.isfinite(rate) or rate <= 0 or int(rate) != rate):
        raise ValueError(f'{name}: camera frequency must be a positive integer in Hz')
    rate = int(rate)
    if render_hz % rate:
        raise ValueError(f'{name}: camera frequency {rate} Hz must divide simulation.render_hz '
                         f'({render_hz} Hz). At 50 Hz use 25, 10 or 5 Hz; '
                         'for a 30 Hz camera use a compatible render_hz such as 60.')
    return rate


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
    parser.add_argument('--effective-world', action='store_true',
                        help='Write validated world YAML, including simulation overrides')
    add_simulation_arguments(parser)
    args = parser.parse_args()

    try:
        world = load_world(args.config, simulation_overrides(args))
        if args.effective_world:
            if not args.output:
                parser.error('--output is required with --effective-world')
            Path(args.output).write_text(yaml.safe_dump(world, sort_keys=False))
        elif args.platform_config:
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
