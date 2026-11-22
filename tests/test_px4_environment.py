"""A swarm must launch each PX4 process with its own namespace snapshot."""
import importlib.util
import os
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class Backend:
    def __init__(self, config):
        self._is_running = False
        self.px4_autolaunch = True
        self.px4_tool = None
        self.px4_dir = '/px4'
        self._vehicle_id = config
        self.px4_vehicle_model = 'iris'

    def start(self):
        self._is_running = True

    def stop(self):
        self._is_running = False
        self.px4_tool = None


class Tool:
    launches = []

    def __init__(self, directory, instance, model):
        self.instance = instance
        self.environment = os.environ
        self.environment['PX4_SIM_MODEL'] = model

    def launch_px4(self):
        self.launches.append((self.instance, self.environment))


class EnvironmentTests(unittest.TestCase):
    def test_independent_namespaces_domain_and_restart(self):
        module1 = types.ModuleType('pegasus.simulator.logic.backends.px4_mavlink_backend')
        module1.PX4MavlinkBackend = Backend
        module2 = types.ModuleType('pegasus.simulator.logic.backends.tools.px4_launch_tool')
        module2.PX4LaunchTool = Tool
        spec = importlib.util.spec_from_file_location('px4_backend', ROOT / 'scripts/px4_backend.py')
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {module1.__name__: module1, module2.__name__: module2}), \
             patch.dict(os.environ, {'ROS_DOMAIN_ID': '10', 'PX4_UXRCE_DDS_NS': 'original'}):
            spec.loader.exec_module(module)
            Tool.launches = []
            drones = [module.NamespacedPX4Backend(i, name) for i, name in enumerate(('leader', 'wingman'))]
            for drone in drones:
                drone.start()
                drone.start()  # no duplicate process when already running
            self.assertEqual(len(Tool.launches), 2)
            self.assertEqual([env['PX4_UXRCE_DDS_NS'] for _, env in Tool.launches], ['leader', 'wingman'])
            self.assertTrue(all(env['ROS_DOMAIN_ID'] == '10' for _, env in Tool.launches))
            self.assertIsNot(Tool.launches[0][1], Tool.launches[1][1])
            self.assertEqual(os.environ['PX4_UXRCE_DDS_NS'], 'original')
            drones[0].stop()
            drones[0].start()
            self.assertEqual(Tool.launches[-1][1]['PX4_UXRCE_DDS_NS'], 'leader')


if __name__ == '__main__':
    unittest.main()
