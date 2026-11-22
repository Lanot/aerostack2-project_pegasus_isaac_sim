#!/usr/bin/env python3
"""Standalone Aerostack2/Pegasus launcher for Isaac Sim 6.x.

Based on the original examples by Marcelo Jacinto (BSD-3-Clause, 2024).
Requires the Isaac Sim 6.x port of Pegasus Simulator.
"""

import argparse
import math
import os
from pathlib import Path

from world_config import PROJECT_ROOT, load_world, payloads


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

    parser.add_argument('--max-steps', type=int, default=0,
                        help='Exit after this many app updates (0: run until closed)')

    args = parser.parse_args(argv)

    if args.max_steps < 0:
        parser.error('max-steps must be nonnegative')
    config_file = args.config or str(PROJECT_ROOT / 'config' / (
        'world_swarm.yaml' if args.swarm else 'world.yaml'))
    try:
        args.world = load_world(config_file)
        # Validate supported payloads before creating the simulator.
        for drone in args.world['drones']:
            drone['_payloads'] = payloads(drone)
    except (ValueError, OSError, __import__('yaml').YAMLError) as error:
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


def create_clock():
    """Publish simulation time for the AS2 nodes configured with use_sim_time."""
    import omni.graph.core as og

    keys = og.Controller.Keys

    return og.Controller.edit(
        {'graph_path': '/World/ROS2Clock', 'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('Tick', 'omni.graph.action.OnPlaybackTick'),
                ('Time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                ('Clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
            ],
            keys.CONNECT: [
                ('Tick.outputs:tick', 'Clock.inputs:execIn'),
                ('Time.outputs:simulationTime', 'Clock.inputs:timeStamp'),
            ],
            keys.SET_VALUES: [('Clock.inputs:topicName', '/clock')],
        },
    )


class FollowCamera:
    """Follow the live physics state, including when USD transform updates are disabled."""

    def __init__(self, drone, headless):
        import omni.usd
        from pxr import UsdGeom

        self.drone = drone

        camera = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), '/World/SimpleFollowCamera')
        camera.GetFocalLengthAttr().Set(30.0)
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(24.0)

        self.transform = UsdGeom.Xformable(camera.GetPrim()).AddTransformOp()

        if not headless:
            from omni.kit.viewport.utility import get_active_viewport
            viewport = get_active_viewport()
            if viewport is not None:
                viewport.camera_path = str(camera.GetPath())

    def update(self):
        from pxr import Gf
        x, y, z = (float(v) for v in self.drone.state.position)
        qx, qy, qz, qw = (float(v) for v in self.drone.state.attitude)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        eye = Gf.Vec3d(x - 2 * math.cos(yaw), y - 2 * math.sin(yaw), z + 0.5)
        view = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(x, y, z), Gf.Vec3d(0, 0, 1))
        self.transform.Set(view.GetInverse())


def create_empty_world(stage):
    """A Z-up ground plane and light, matching Gazebo's empty world behavior."""
    from pxr import UsdGeom, UsdPhysics, UsdLux, Gf
    ground = UsdGeom.Mesh.Define(stage, '/World/Ground')
    ground.CreatePointsAttr([(-250, -250, 0), (250, -250, 0), (250, 250, 0), (-250, 250, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(800.0)


def set_camera_view(stage, view, headless):
    from pxr import UsdGeom, Gf
    camera = UsdGeom.Camera.Define(stage, '/World/SceneCamera')
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['eye']), Gf.Vec3d(*view['target']), Gf.Vec3d(0, 0, 1))
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(matrix.GetInverse())
    if not headless:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = str(camera.GetPath())


def build_scene(app, args, following=False):
    # Kit modules must only be imported after SimulationApp starts.
    import omni.timeline
    import omni.usd
    from pxr import UsdGeom
    from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
    from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

    context = omni.usd.get_context()
    if args.world_usd:
        if not context.open_stage(args.world_usd):
            raise RuntimeError(f'Cannot open world stage: {args.world_usd}')
    else:
        context.new_stage()
    while context.get_stage_loading_status()[2] > 0:
        app.update()

    stage = context.get_stage()
    if args.world_usd:
        if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z or not math.isclose(
            UsdGeom.GetStageMetersPerUnit(stage), 1.0
        ):
            raise ValueError('Pegasus requires a Z-up world measured in meters')
    else:
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    pg = PegasusInterface()
    pg.initialize_world()

    if not args.world_usd and args.environment.lower() not in ('none', 'empty'):
        if args.environment not in SIMULATION_ENVIRONMENTS:
            raise ValueError(f'Unknown environment {args.environment!r}; '
                             f'choose from {list(SIMULATION_ENVIRONMENTS)}')
        # load_asset raises if the reference fails; load_environment only logs the error.
        pg.load_asset(SIMULATION_ENVIRONMENTS[args.environment], '/World/layout')
        while context.get_stage_loading_status()[2] > 0:
            app.update()

    if args.world.get('origin'):
        pg.set_global_coordinates(**args.world['origin'])
    if args.environment.lower() == 'empty' and not args.world_usd:
        create_empty_world(stage)
    if not args.no_ros2:
        create_clock()

    drones = [spawn_vehicle(app, args, spec, following, context) for spec in args.world['drones']]
    by_name = dict(zip((d['model_name'] for d in args.world['drones']), drones))
    follow_camera = FollowCamera(by_name[args.follow_drone], args.headless) if following else None
    if not following and args.world.get('scene_camera_view'):
        set_camera_view(stage, args.world['scene_camera_view'], args.headless)
    return drones, follow_camera


def spawn_vehicle(app, args, spec, following, context):
    import omni.usd
    from pegasus.simulator.params import ROBOTS
    from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
    config = MultirotorConfig()
    config.backends = []

    if not args.no_px4:
        from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackendConfig
        from px4_backend import NamespacedPX4Backend
        config.backends.append(NamespacedPX4Backend(PX4MavlinkBackendConfig({
            'vehicle_id': spec['instance'],
            'px4_autolaunch': not args.no_px4_autolaunch,
            'px4_dir': str(args.px4_dir),
            'px4_vehicle_model': args.px4_model,
        }), namespace=spec.get('dds_namespace', spec['model_name'])))

    if not args.no_ros2:
        from isaacsim_ros2 import IsaacSim6ROS2Backend
        config.backends.append(IsaacSim6ROS2Backend(vehicle_id=spec['instance'], config={
            'namespace_exact': spec.get('sensor_namespace', spec['model_name']),
            'pub_sensors': True,
            'pub_graphical_sensors': True, 'pub_state': False, 'sub_control': False,
        }))

    config.graphical_sensors = []
    for sensor in spec['_payloads']:
        if sensor['model_type'] not in ('hd_camera', 'camera', 'monocular_camera') or args.no_camera:
            continue
        from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
        # The Pegasus API uses frequency, not update_rate.
        camera_config = {'frequency': 60}
        if following:
            import numpy as np
            camera_config.update({
                'resolution': (1920, 1080),
                'intrinsics': np.array([[609.25, 0, 960], [0, 609.25, 540], [0, 0, 1]]),
                'distortion_coefficients': np.array([
                    5.61802924772665, 2.633097129851353, -1.839432678176205e-5,
                    -4.505550891889316e-4, -16.34711859741078, 5.107694839223768,
                    3.699287666569801, -16.68676891240623, 0, 0, 0, 0, 0, 0,
                ]),
            })
        camera_config.update(sensor.get('config', {}))
        config.graphical_sensors.append(MonocularCamera(sensor.get('model_name', 'camera'), config=camera_config))

    for sensor in spec['_payloads']:
        if sensor['model_type'] not in ('lidar', 'lidar3d', 'gpu_lidar') or args.no_lidar:
            continue
        from pegasus.simulator.logic.graphical_sensors.lidar import Lidar as PegasusLidar
        from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
        from isaacsim.sensors.experimental.rtx import Lidar

        class IsaacSim6Lidar(PegasusLidar):
            """Keep Pegasus sensor scheduling with Isaac Sim 6's RTX authoring API."""

            def __init__(self, name, settings):
                super().__init__(name, config=settings)
                self.sensor_config = settings.get('sensor_configuration', 'Example_Rotary')

            def initialize(self, vehicle):
                GraphicalSensor.initialize(self, vehicle)
                self._stage_prim_path = omni.usd.get_stage_next_free_path(
                    context.get_stage(), vehicle.prim_path + '/body/' + self._lidar_name, False
                )
                self._lidar_name = self._stage_prim_path.rsplit('/', 1)[-1]
                qx, qy, qz, qw = self._orientation
                self._sensor = Lidar.create(
                    path=self._stage_prim_path, config=self.sensor_config,
                    translations=[self._position], orientations=[[qw, qx, qy, qz]],
                )

            def start(self):
                # ROS2Backend owns the render product and ROS writers.
                pass

        config.graphical_sensors.append(IsaacSim6Lidar(sensor.get('model_name', 'lidar'), sensor.get('config', {})))

    from scipy.spatial.transform import Rotation
    model = spec.get('pegasus_model', spec['model_type'])
    model = {'quadrotor_base': 'Iris', 'iris': 'Iris'}.get(model, model)
    if not spec.get('usd_path') and model not in ROBOTS:
        raise ValueError(f'Unsupported Pegasus drone model {model!r}; set pegasus_model or usd_path')
    return Multirotor('/World/' + spec['model_name'], spec.get('usd_path') or ROBOTS[model],
                      spec['instance'], spec['xyz'],
                      Rotation.from_euler('xyz', spec['rpy']).as_quat(), config=config)


def main(argv=None):
    args = parse_arguments(argv)

    if args.print_config:
        import json
        print(json.dumps(args.world, indent=2))
        return
    following = args.follow_camera
    from isaacsim import SimulationApp
    app = SimulationApp({
        'headless': args.headless,
        'extra_args': ['--ext-folder', str(args.pegasus_dir / 'extensions')],
    })
    timeline = None

    try:
        import omni.kit.app
        import omni.timeline

        manager = omni.kit.app.get_app().get_extension_manager()
        extensions = ['pegasus.simulator']
        if not args.no_camera:
            extensions.append('isaacsim.sensors.camera')
        if not args.no_lidar:
            extensions.append('isaacsim.sensors.experimental.rtx')
        if not args.no_ros2:
            extensions.append('isaacsim.ros2.bridge')
        for extension in extensions:
            manager.set_extension_enabled_immediate(extension, True)
            if not manager.is_extension_enabled(extension):
                raise RuntimeError(f'Could not enable {extension}')

        app.update()

        timeline = omni.timeline.get_timeline_interface()
        drones, follow_camera = build_scene(app, args, following=following)
        timeline.play()
        steps = 0

        while app.is_running() and (not args.max_steps or steps < args.max_steps):
            app.update()
            if follow_camera is not None:
                follow_camera.update()
            steps += 1
        print(f'PEGASUS_RUN_COMPLETE steps={steps}', flush=True)

    finally:
        try:
            if timeline is not None:
                timeline.stop()
                # Dispatch Pegasus stop callbacks before shutting Kit down.
                for _ in range(3):
                    if app.is_running():
                        app.update()
        finally:
            app.close()


if __name__ == '__main__':
    main()
