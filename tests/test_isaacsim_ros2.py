"""Check sensor helper wiring without requiring a running Kit application."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1] / 'scripts'


class SensorGraphTests(unittest.TestCase):
    def setUp(self):
        omni = types.ModuleType('omni')
        omni.graph = types.ModuleType('omni.graph')
        omni.graph.core = MagicMock()
        omni.replicator = types.ModuleType('omni.replicator')
        omni.replicator.core = MagicMock()
        backend = types.ModuleType('pegasus.simulator.logic.backends.ros2_backend')
        class FakeBackend:
            def __init__(self, vehicle_id, num_rotors, config):
                self._namespace = config.get('namespace', 'drone')
                self._id = vehicle_id
                self.initialize_publishers(config)

            def initialize_publishers(self, config):
                self.captured_topic_namespace = self._namespace + str(self._id)

        backend.ROS2Backend = FakeBackend
        modules = {'omni': omni, 'omni.graph': omni.graph,
                   'omni.graph.core': omni.graph.core, 'omni.replicator': omni.replicator,
                   'omni.replicator.core': omni.replicator.core,
                   'pegasus.simulator.logic.backends.ros2_backend': backend}
        spec = importlib.util.spec_from_file_location('isaacsim_ros2', ROOT / 'isaacsim_ros2.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules):
            spec.loader.exec_module(module)
        self.module = module
        self.og = omni.graph.core
        self.rep = omni.replicator.core
        self.backend = module.IsaacSim6ROS2Backend.__new__(module.IsaacSim6ROS2Backend)
        self.backend._namespace = 'drone'
        self.backend._id = 1
        self.backend.graphical_sensors_writers = {}

    def graph_values(self):
        config = self.og.Controller.edit.call_args.args[1]
        return dict(config[self.og.Controller.Keys.SET_VALUES])

    def test_camera_info_comes_from_render_product_and_depth_is_optional(self):
        camera = types.SimpleNamespace(_render_product_path='/Render/Camera')
        data = {'camera_name': 'camera', 'stage_prim_path': '/World/drone/camera',
                'camera': camera}
        self.backend.add_monocular_camera_writter(data)
        values = self.graph_values()
        self.assertEqual(values['Info.inputs:renderProductPath'], '/Render/Camera')
        self.assertEqual(values['RGB.inputs:nodeNamespace'], 'drone1')
        self.assertEqual(values['RGB.inputs:topicName'], 'camera/color/image_raw')
        self.assertNotIn('Depth.inputs:type', values)
        self.backend.add_monocular_camera_writter(dict(data, depth=True))
        self.assertEqual(self.graph_values()['Depth.inputs:type'], 'depth')
        self.assertIn('camera', self.backend.graphical_sensors_writers)

    def test_arbitrary_drone_name_is_exact_and_numeric_id_is_preserved(self):
        backend = self.module.IsaacSim6ROS2Backend(4, config={'namespace_exact': 'leader'})
        self.assertEqual(backend.captured_topic_namespace, 'leader')
        self.assertEqual(backend._id, 4)

    def test_lidar_publishes_both_types_and_retains_render_product(self):
        product = types.SimpleNamespace(path='/Render/Lidar')
        self.rep.create.render_product.return_value = product
        self.backend.add_lidar_writter({
            'lidar_name': 'lidar', 'stage_prim_path': '/World/drone/lidar',
        })
        values = self.graph_values()
        self.assertEqual(values['Points.inputs:type'], 'point_cloud')
        self.assertEqual(values['Scan.inputs:type'], 'laser_scan')
        self.assertFalse(values['Scan.inputs:useSystemTime'])
        self.assertIs(self.backend.graphical_sensors_writers['lidar'][1], product)


if __name__ == '__main__':
    unittest.main()
