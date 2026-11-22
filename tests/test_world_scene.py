"""Exercise the real world-spawning path with Kit/Pegasus API doubles."""
import importlib.util
from pathlib import Path
import sys
import types
import tempfile
import yaml
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import isaacsim_app as launcher


class SceneTests(unittest.TestCase):
    def test_swarm_uses_distinct_paths_positions_instances_and_one_clock(self):
        world = yaml.safe_load((ROOT / 'config/world_swarm.yaml').read_text())
        world.update(px4={'enabled': False}, ros2=False)
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'world.yaml'
            config.write_text(yaml.safe_dump(world))
            args = launcher.parse_arguments(['--config', str(config), '--print-config'])
        context = MagicMock()
        context.get_stage_loading_status.return_value = (0, 0, 0)
        omni = types.ModuleType('omni')
        omni.usd = types.ModuleType('omni.usd')
        omni.usd.get_context = MagicMock(return_value=context)
        omni.timeline = types.ModuleType('omni.timeline')
        pxr = types.ModuleType('pxr')
        pxr.UsdGeom = MagicMock()
        pxr.Gf = MagicMock()
        pxr.UsdLux = MagicMock()
        pxr.UsdPhysics = MagicMock()
        omni.graph = types.ModuleType("omni.graph")
        graph = types.ModuleType("omni.graph.core")
        omni.graph.core = graph
        viewport = types.ModuleType("omni.kit.viewport.utility")
        viewport.get_active_viewport = MagicMock()
        params = types.ModuleType('pegasus.simulator.params')
        params.ROBOTS = {'Iris': '/assets/iris.usd'}
        params.SIMULATION_ENVIRONMENTS = {}
        interface = types.ModuleType('pegasus.simulator.logic.interface.pegasus_interface')
        interface.PegasusInterface = MagicMock()
        vehicles = types.ModuleType('pegasus.simulator.logic.vehicles.multirotor')
        vehicles.MultirotorConfig = lambda: types.SimpleNamespace()
        vehicles.Multirotor = MagicMock()
        modules = {'omni.graph': omni.graph, 'omni.graph.core': graph, 'omni.kit.viewport.utility': viewport, 'omni': omni, 'omni.usd': omni.usd, 'omni.timeline': omni.timeline,
                   'pxr': pxr, params.__name__: params, interface.__name__: interface,
                   vehicles.__name__: vehicles}
        with patch.dict(sys.modules, modules):
            spec = importlib.util.spec_from_file_location('isaacsim_scene', ROOT / 'scripts/isaacsim_scene.py')
            scene = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(scene)
            with patch.object(scene, 'create_empty_world') as ground, \
                 patch.object(scene, 'create_clock') as clock:
                drones, follow = scene.build_scene(MagicMock(), args)
        self.assertEqual(len(drones), 3)
        calls = vehicles.Multirotor.call_args_list
        self.assertEqual([c.args[0] for c in calls], ['/World/drone0', '/World/drone1', '/World/drone2'])
        self.assertEqual([c.args[2] for c in calls], [0, 1, 2])
        self.assertEqual([c.args[3] for c in calls], [[-2., 0., .2], [2., 0., .2], [0., 0., .2]])
        self.assertEqual(len({id(c.kwargs['config']) for c in calls}), 3)
        interface.PegasusInterface.return_value.set_global_coordinates.assert_called_once_with(
            latitude=40.337494, longitude=-3.883197, altitude=100.)
        ground.assert_called_once()
        clock.assert_not_called()
        self.assertIsNone(follow)


if __name__ == '__main__':
    unittest.main()
