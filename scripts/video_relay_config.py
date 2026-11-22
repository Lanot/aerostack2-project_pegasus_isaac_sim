"""Plan per-camera ROS topics and UDP ports without importing Isaac Sim or ROS."""
import argparse
import json
from pathlib import Path
import re
import sys

import yaml

from world_config import load_world, payloads

CAMERA_TYPES = {'camera', 'hd_camera', 'monocular_camera'}


def video_relays(world, host='127.0.0.1'):
    routes, used_ports = [], {}
    for drone in world['drones']:
        name = drone['model_name']
        cameras = [p for p in payloads(drone, warn_gimbals=False) if p['model_type'] in CAMERA_TYPES]

        if not cameras:
            continue
        if not world['ros2']:
            raise ValueError('Video relays require ros2: true to publish camera images')
        if len(cameras) > 10:
            raise ValueError(f'{name}: at most 10 camera relays fit in its reserved UDP port block')

        match = re.fullmatch(r'drone([0-9]+)', name)
        number = int(match.group(1)) if match else drone['instance']
        base_port = 5600 + 10 * number
        if base_port + 9 > 65535:
            raise ValueError(f'{name}: UDP port block exceeds 65535')

        camera_names = set()
        namespace = drone.get('sensor_namespace', name)
        if not isinstance(namespace, str):
            raise ValueError(f'{name}: sensor_namespace must be a string')
        namespace = namespace.strip('/')
        if not namespace:
            raise ValueError(f'{name}: sensor_namespace must not be empty')
        for index, camera in enumerate(cameras):
            camera_name = camera.get('model_name', 'camera')
            if not isinstance(camera_name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', camera_name):
                raise ValueError(f'{name}: cameras need valid ROS model_name values')
            if camera_name in camera_names:
                raise ValueError(f'{name}: duplicate camera name {camera_name!r}')
            camera_names.add(camera_name)
            port = base_port + index
            if port in used_ports:
                raise ValueError(f'UDP port {port} collision between {used_ports[port]} and {name}')
            used_ports[port] = name
            routes.append(dict(drone=name, camera=camera_name,
                               topic=f'/{namespace}/{camera_name}/color/image_raw',
                               host=host, port=port))
    return routes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--drone', required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    try:
        world = load_world(args.config)
        if args.drone not in [d['model_name'] for d in world['drones']]:
            raise ValueError(f'Unknown drone: {args.drone}')
        routes = [route for route in video_relays(world, args.host) if route['drone'] == args.drone]
        Path(args.output).write_text(json.dumps(routes, indent=2) + '\n')
        for route in routes:
            print(f"[WARN] [video-relay] {route['drone']}/{route['camera']}: "
                  f"{route['topic']} -> udp://{route['host']}:{route['port']}", file=sys.stderr, flush=True)
        if not routes:
            print(f'[WARN] [video-relay] {args.drone}: no camera payloads; no relays started',
                  file=sys.stderr, flush=True)
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
