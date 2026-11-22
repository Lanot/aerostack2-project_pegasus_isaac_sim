"""Scene implementation; loaded after SimulationApp and extension startup.

Based on the original examples by Marcelo Jacinto (BSD-3-Clause, 2024).
Optional sensor/back-end imports remain conditional so physics-only runs do not
require unused extensions.
"""
import math

from callback_profiler import profiled_vehicle_class, profile_backend
from world_config import camera_frequency

import numpy as np
from scipy.spatial.transform import Rotation

import carb
import omni.graph.core as og
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
import omni.usd
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig


FOLLOWING_CAMERA_PARAMS = {
    'resolution': (1920, 1080),
    'intrinsics': np.array([[609.25, 0, 960], [0, 609.25, 540], [0, 0, 1]]),
    'distortion_coefficients': np.array([
        5.61802924772665, 2.633097129851353, -1.839432678176205e-5,
        -4.505550891889316e-4, -16.34711859741078, 5.107694839223768,
        3.699287666569801, -16.68676891240623, 0, 0, 0, 0, 0, 0,
    ])
}

def create_clock():
    """Publish simulation time for the AS2 nodes configured with use_sim_time."""

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

        self.drone = drone

        camera = UsdGeom.Camera.Define(omni.usd.get_context().get_stage(), '/World/SimpleFollowCamera')
        camera.GetFocalLengthAttr().Set(30.0)
        camera.GetHorizontalApertureAttr().Set(36.0)
        camera.GetVerticalApertureAttr().Set(24.0)

        self.transform = UsdGeom.Xformable(camera.GetPrim()).AddTransformOp()

        if not headless:
            viewport = get_active_viewport()
            if viewport is not None:
                viewport.camera_path = str(camera.GetPath())

    def update(self):
        x, y, z = (float(v) for v in self.drone.state.position)
        qx, qy, qz, qw = (float(v) for v in self.drone.state.attitude)
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        eye = Gf.Vec3d(x - 2 * math.cos(yaw), y - 2 * math.sin(yaw), z + 0.5)
        view = Gf.Matrix4d().SetLookAt(eye, Gf.Vec3d(x, y, z), Gf.Vec3d(0, 0, 1))
        self.transform.Set(view.GetInverse())


def create_empty_world(stage):
    """A Z-up ground plane and light, matching Gazebo's empty world behavior."""
    ground = UsdGeom.Mesh.Define(stage, '/World/Ground')
    ground.CreatePointsAttr([(-250, -250, 0), (250, -250, 0), (250, 250, 0), (-250, 250, 0)])
    ground.CreateFaceVertexCountsAttr([4])
    ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground.CreateSubdivisionSchemeAttr('none')
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(800.0)


def set_camera_view(stage, view, headless):
    camera = UsdGeom.Camera.Define(stage, '/World/SceneCamera')
    matrix = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*view['eye']), Gf.Vec3d(*view['target']), Gf.Vec3d(0, 0, 1))
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(matrix.GetInverse())
    if not headless:
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.camera_path = str(camera.GetPath())


def build_scene(app, args, following=False):
    context = omni.usd.get_context()
    if args.world_usd:
        print(f'[AS2] Opening world USD: {args.world_usd}', flush=True)
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

    # Physics rate alone does not configure Kit's timeline/run-loop cadence.
    rates = args.world['simulation']
    settings = carb.settings.get_settings()
    settings.set_bool('/app/runLoops/main/rateLimitEnabled', True)
    # Allow the full physics-step budget for each application tick.
    settings.set_float('/persistent/simulation/minFrameRate', float(rates['render_hz']))
    SimulationManager.setup_simulation(dt=1.0 / rates['physics_hz'])
    RenderingManager.set_dt(1.0 / rates['render_hz'])
    print(f"SIM_TIMING physics_hz={rates['physics_hz']} render_hz={rates['render_hz']} "
          f"physics_steps_per_update={rates['physics_hz'] / rates['render_hz']:.2f}", flush=True)

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

    drones = []
    for spec in args.world['drones']:
        print(f"[AS2] Spawning {spec['model_name']} (PX4 instance {spec['instance']})", flush=True)
        drones.append(spawn_vehicle(app, args, spec, following, context))
        print(f"[AS2] Spawned {spec['model_name']}", flush=True)

    by_name = dict(zip((d['model_name'] for d in args.world['drones']), drones))
    follow_camera = FollowCamera(by_name[args.follow_drone], args.headless) if following else None
    if not following and args.world.get('scene_camera_view'):
        set_camera_view(stage, args.world['scene_camera_view'], args.headless)
    return drones, follow_camera


def spawn_vehicle(app, args, spec, following, context):
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
            'enable_lockstep': args.world['px4']['enable_lockstep'],
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
        camera_config = {'frequency': camera_frequency(
            sensor, args.world['simulation']['render_hz'], spec['model_name'])}
        if following:
            camera_config.update(FOLLOWING_CAMERA_PARAMS)

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

    model = spec.get('pegasus_model', spec['model_type'])
    model = {'quadrotor_base': 'Iris', 'iris': 'Iris'}.get(model, model)
    if not spec.get('usd_path') and model not in ROBOTS:
        raise ValueError(f'Unsupported Pegasus drone model {model!r}; set pegasus_model or usd_path')
    vehicle_type = Multirotor
    profiler = getattr(args, 'callback_profiler', None)
    if profiler is not None:
        vehicle_type = profiled_vehicle_class(Multirotor, profiler, spec['model_name'])
        for backend in config.backends:
            profile_backend(backend, profiler, spec['model_name'])
    return vehicle_type('/World/' + spec['model_name'], spec.get('usd_path') or ROBOTS[model],
                      spec['instance'], spec['xyz'],
                      Rotation.from_euler('xyz', spec['rpy']).as_quat(), config=config)




def simulation_counters():
    """Physics time/step counters, independent of viewport and ROS publication rates."""
    return SimulationManager.get_simulation_time(), SimulationManager.get_num_physics_steps()
