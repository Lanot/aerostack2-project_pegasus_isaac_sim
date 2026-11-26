#!/usr/bin/env python3
"""Standalone Aerostack2/Pegasus launcher for Isaac Sim 6.x.

Based on the original examples by Marcelo Jacinto (BSD-3-Clause, 2024).
Requires the Isaac Sim 6.x port of Pegasus Simulator.
"""

import argparse
from importlib import import_module
import json
import os
import sys
import traceback
from pathlib import Path

import yaml

from callback_profiler import CallbackProfiler
from simulation_performance import SimulationPerformance
from world_config import (PROJECT_ROOT, add_simulation_arguments, load_world,
                          payloads, simulation_overrides)
from simulation_settings import apply_simulation_settings


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--px4-dir', '--px4_dir', dest='px4_dir',
                        default=os.environ.get('PX4_DIR', '~/PX4-Autopilot'))
    parser.add_argument('--pegasus-dir', default=os.environ.get(
        'PEGASUS_SIMULATOR_DIR', '~/IsaacSim-Extensions/PegasusSimulator'))

    parser.add_argument('-m', '--swarm', action='store_true', help='Use config/world_swarm.yaml')

    parser.add_argument('--config', '--simulation-config-file', help='World YAML (default: config/world.yaml)')
    parser.add_argument('--print-config', action='store_true', help='Validate and print config without starting Kit')

    parser.add_argument('--follow-camera', action=argparse.BooleanOptionalAction, default=None,
                        help='Override world follow_camera (otherwise use YAML)')
    parser.add_argument('--follow-drone', help='Override world follow_drone')

    parser.add_argument('--headless', action='store_true')
    add_simulation_arguments(parser)

    parser.add_argument('--max-steps', type=int, default=0,
                        help='Exit after this many app updates (0: run until closed)')

    args = parser.parse_args(argv)

    if args.max_steps < 0:
        parser.error('max-steps must be nonnegative')
    config_file = args.config or str(PROJECT_ROOT / 'config' / (
        'world_swarm.yaml' if args.swarm else 'world.yaml'))
    try:
        args.world = load_world(config_file, simulation_overrides(args))
        # Validate supported payloads before creating the simulator.
        for drone in args.world['drones']:
            drone['_payloads'] = payloads(drone)
    except (ValueError, OSError, yaml.YAMLError) as error:
        parser.error(str(error))

    # Scenario settings belong exclusively to the validated world file.
    world = args.world
    args.environment = world.get('environment', world['world_name'])
    args.world_usd = world.get('world_usd')

    if args.follow_camera is not None:
        world['follow_camera'] = args.follow_camera

    if args.follow_drone is not None:
        if args.follow_drone not in [d['model_name'] for d in world['drones']]:
            parser.error('--follow-drone must match a configured model_name')
        world['follow_drone'] = args.follow_drone

    args.follow_camera = world['follow_camera']
    args.follow_drone = world.get('follow_drone', world['drones'][0]['model_name'])

    args.no_px4 = not world['px4']['enabled']
    args.no_px4_autolaunch = not world['px4']['autolaunch']
    args.px4_model = world['px4']['model']

    args.no_ros2 = not world['ros2']

    kinds = {sensor['model_type'] for drone in world['drones'] for sensor in drone['_payloads']}

    args.no_camera = not kinds.intersection(('hd_camera', 'camera', 'monocular_camera'))
    args.no_lidar = not kinds.intersection(('lidar', 'lidar3d', 'gpu_lidar'))
    if args.print_config:
        return args

    args.pegasus_dir = Path(args.pegasus_dir).expanduser().resolve()

    if not (args.pegasus_dir / 'extensions/pegasus.simulator/config/extension.toml').is_file():
        parser.error(f'Pegasus extension not found under {args.pegasus_dir}/extensions')
    args.px4_dir = Path(args.px4_dir).expanduser().resolve()

    if not args.no_px4 and not args.no_px4_autolaunch:
        binary = args.px4_dir / 'build/px4_sitl_default/bin/px4'
        if not binary.is_file():
            parser.error(f'Build PX4 SITL first: missing {binary}')

    return args


def main(argv=None):
    args = parse_arguments(argv)

    if args.print_config:
        print(json.dumps(args.world, indent=2))
        return

    following = args.follow_camera

    # Isaac Sim and Kit are unavailable during offline CLI validation.
    SimulationApp = import_module('isaacsim').SimulationApp
    extra_args = ['--ext-folder', str(args.pegasus_dir / 'extensions')]

    for directory in args.world['extension_paths']:
        extra_args.extend(['--ext-folder', directory])
    if 'cesium.omniverse' in args.world['extensions']:
        # Register schemas at Kit startup, and use Fabric for streamed tile data.
        # The main Cesium extension is enabled after SimulationApp returns.
        extra_args.extend(['--enable', 'cesium.usd.plugins',
                           '--/app/useFabricSceneDelegate=true',
                           '--/exts/cesium.omniverse/showOnStartup=false'])

    app = SimulationApp({
        'headless': args.headless,
        "scene_graph_instancing": True,
        'disable_viewport_updates': args.headless,
        'extra_args': extra_args,
    })

    timeline = None
    exit_code = 0
    phase = 'enabling extensions'

    try:
        kit_app = import_module('omni.kit.app')
        timeline_module = import_module('omni.timeline')

        manager = kit_app.get_app().get_extension_manager()
        extensions = ['pegasus.simulator']
        if not args.no_camera:
            extensions.append('isaacsim.sensors.camera')
        if not args.no_lidar:
            extensions.append('isaacsim.sensors.experimental.rtx')
        if not args.no_ros2:
            extensions.append('isaacsim.ros2.bridge')

        extensions.extend(args.world['extensions'])

        for extension in dict.fromkeys(extensions):
            print(f'[AS2] Enabling extension: {extension}', flush=True)
            manager.set_extension_enabled_immediate(extension, True)
            if not manager.is_extension_enabled(extension):
                raise RuntimeError(f'Could not enable {extension}')

        apply_simulation_settings(import_module('carb').settings.get_settings(),
                                  args.world['simulation'])
        app.update()

        timeline = timeline_module.get_timeline_interface()
        # Load scene dependencies only after Kit and the requested extensions.
        phase = 'loading scene module'
        scene = import_module('isaacsim_scene')
        args.callback_profiler = (CallbackProfiler() if args.world.get('simulation', {}).get('profile', False) else None)
        phase = 'building world and spawning vehicles'
        drones, follow_camera = scene.build_scene(app, args, following=following)
        phase = 'starting physics'
        print('[AS2] Starting physics', flush=True)
        timeline.play()
        performance = (SimulationPerformance(scene.simulation_counters)
                       if args.callback_profiler is not None else None)
        steps = 0

        phase = 'running simulation'
        print('[AS2] Entering simulation loop', flush=True)
        while app.is_running() and (not args.max_steps or steps < args.max_steps):
            if args.callback_profiler is None:
                app.update()
            else:
                args.callback_profiler.call('app.update', app.update)
            if follow_camera is not None:
                follow_camera.update()
            steps += 1
            if performance is not None:
                performance.sample()
            if args.callback_profiler is not None:
                args.callback_profiler.sample()
        print(f'PEGASUS_RUN_COMPLETE steps={steps}', flush=True)

    except KeyboardInterrupt:
        exit_code = 130
        print(f'[AS2] Interrupted during {phase}', file=sys.stderr, flush=True)
        raise

    except Exception:
        exit_code = 1
        # Kit fast shutdown can exit before Python's uncaught-exception handler.
        print(f'[AS2] Simulation failed during {phase}:', file=sys.stderr, flush=True)
        traceback.print_exc()
        sys.stderr.flush()
        raise

    finally:
        try:
            if timeline is not None:
                timeline.stop()
                # Dispatch Pegasus stop callbacks before shutting Kit down.
                for _ in range(3):
                    if app.is_running():
                        app.update()
        except Exception:
            exit_code = 1
            print('[AS2] Simulation cleanup failed:', file=sys.stderr, flush=True)
            traceback.print_exc()
            sys.stderr.flush()
            raise
        finally:
            app.close(exit_code=exit_code)


if __name__ == '__main__':
    main()
