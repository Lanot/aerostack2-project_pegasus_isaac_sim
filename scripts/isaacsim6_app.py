#!/usr/bin/env python3
"""Standalone Aerostack2/Pegasus launcher for Isaac Sim 6.x.

Based on the original examples by Marcelo Jacinto (BSD-3-Clause, 2024).
Requires the Isaac Sim 6.x port of Pegasus Simulator.
"""

import argparse
import math
import os
from pathlib import Path


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument('--px4_dir', '--px4-dir', dest='px4_dir',
                        default=os.environ.get('PX4_DIR', '~/PX4-Autopilot'))
    parser.add_argument('--pegasus-dir', default=os.environ.get(
        'PEGASUS_SIMULATOR_DIR', '~/IsaacSim-Extensions/PegasusSimulator'))
    parser.add_argument('--world-usd', help='Custom USD stage, opened before spawning the drone')
    parser.add_argument('--environment', default='Warehouse with Shelves',
                        help='Pegasus environment name, or none for an empty stage')
    parser.add_argument('--position', type=float, nargs=3, default=[0.0, 0.0, 0.07],
                        metavar=('X', 'Y', 'Z'))
    parser.add_argument('--vehicle-id', type=int, default=0, help='PX4 SITL instance ID')
    parser.add_argument('--ros-vehicle-id', type=int, default=1,
                        help='Sensor namespace suffix (default: drone1)')
    parser.add_argument('--namespace', default='drone', help='Sensor namespace prefix')
    parser.add_argument('--px4-dds-namespace', default='drone1',
                        help='PX4 DDS namespace; match AS2 fmu_prefix')
    parser.add_argument('--px4-model', default='gazebo-classic_iris')
    parser.add_argument('--no-px4', action='store_true', help='No MAVLink backend or PX4 process')
    parser.add_argument('--no-px4-autolaunch', action='store_true',
                        help='Connect to an externally launched PX4 SITL')
    parser.add_argument('--no-ros2', action='store_true', help='Disable sensor publishers and /clock')
    parser.add_argument('--no-camera', action='store_true')
    parser.add_argument('--no-lidar', action='store_true')
    parser.add_argument('--lidar-config', default='Example_Rotary')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--max-steps', type=int, default=0,
                        help='Exit after this many app updates (0: run until closed)')

    args = parser.parse_args(argv)

    if args.vehicle_id < 0 or args.ros_vehicle_id < 0 or args.max_steps < 0:
        parser.error('vehicle IDs and max-steps must be nonnegative')
    args.pegasus_dir = Path(args.pegasus_dir).expanduser().resolve()

    if not (args.pegasus_dir / 'extensions/pegasus.simulator/config/extension.toml').is_file():
        parser.error(f'Pegasus extension not found under {args.pegasus_dir}/extensions')
    args.px4_dir = Path(args.px4_dir).expanduser().resolve()

    if not args.no_px4 and not args.no_px4_autolaunch:
        binary = args.px4_dir / 'build/px4_sitl_default/bin/px4'
        if not binary.is_file():
            parser.error(f'Build PX4 SITL first: missing {binary}')

    if args.world_usd and '://' not in args.world_usd:
        args.world_usd = str(Path(args.world_usd).expanduser().resolve())
        if not Path(args.world_usd).is_file():
            parser.error(f'World USD not found: {args.world_usd}')

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

    if not args.world_usd and args.environment.lower() != 'none':
        if args.environment not in SIMULATION_ENVIRONMENTS:
            raise ValueError(f'Unknown environment {args.environment!r}; '
                             f'choose from {list(SIMULATION_ENVIRONMENTS)}')
        # load_asset raises if the reference fails; load_environment only logs the error.
        pg.load_asset(SIMULATION_ENVIRONMENTS[args.environment], '/World/layout')
        while context.get_stage_loading_status()[2] > 0:
            app.update()

    config = MultirotorConfig()
    config.backends = []

    if not args.no_px4:
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend, PX4MavlinkBackendConfig,
        )
        config.backends.append(PX4MavlinkBackend(PX4MavlinkBackendConfig({
            'vehicle_id': args.vehicle_id,
            'px4_autolaunch': not args.no_px4_autolaunch,
            'px4_dir': str(args.px4_dir),
            'px4_vehicle_model': args.px4_model,
        })))

    if not args.no_ros2:
        from isaacsim6_ros2 import IsaacSim6ROS2Backend
        config.backends.append(IsaacSim6ROS2Backend(vehicle_id=args.ros_vehicle_id, config={
            'namespace': args.namespace, 'pub_sensors': following,
            'pub_graphical_sensors': True, 'pub_state': False, 'sub_control': False,
        }))
        create_clock()

    config.graphical_sensors = []
    if not args.no_camera:
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
        config.graphical_sensors.append(MonocularCamera('camera', config=camera_config))

    if not args.no_lidar:
        from pegasus.simulator.logic.graphical_sensors.lidar import Lidar as PegasusLidar
        from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
        from isaacsim.sensors.experimental.rtx import Lidar

        class IsaacSim6Lidar(PegasusLidar):
            """Keep Pegasus sensor scheduling with Isaac Sim 6's RTX authoring API."""

            def initialize(self, vehicle):
                GraphicalSensor.initialize(self, vehicle)
                self._stage_prim_path = omni.usd.get_stage_next_free_path(
                    context.get_stage(), vehicle.prim_path + '/body/' + self._lidar_name, False
                )
                self._lidar_name = self._stage_prim_path.rsplit('/', 1)[-1]
                qx, qy, qz, qw = self._orientation
                self._sensor = Lidar.create(
                    path=self._stage_prim_path, config=args.lidar_config,
                    translations=[self._position], orientations=[[qw, qx, qy, qz]],
                )

            def start(self):
                # ROS2Backend owns the render product and ROS writers.
                pass

        config.graphical_sensors.append(IsaacSim6Lidar('lidar'))

    drone = Multirotor('/World/quadrotor', ROBOTS['Iris'], args.vehicle_id,
                       args.position, [0.0, 0.0, 0.0, 1.0], config=config)
    follow_camera = FollowCamera(drone, args.headless) if following else None

    return drone, follow_camera


def main(argv=None, following=False):
    args = parse_arguments(argv)

    # PX4's rcS reads this when Pegasus autostarts SITL. No separate SITL launch needed.
    if not args.no_px4 and not args.no_px4_autolaunch:
        os.environ['PX4_UXRCE_DDS_NS'] = args.px4_dds_namespace
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
        drone, follow_camera = build_scene(app, args, following=following)
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
