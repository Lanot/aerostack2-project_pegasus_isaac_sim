"""Offline launcher regressions; GPU/sensor/flight checks are documented in README."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import types
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('isaacsim6_app', ROOT / 'isaacsim6_app.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_both_entrypoints_help_without_isaacsim(self):
        for script in ('drone_camera_lidar_isaac_sim.py',
                       'drone_camera_lidar_isaac_sim_following.py'):
            result = subprocess.run([sys.executable, str(ROOT / script), '--help'],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('--world-usd', result.stdout)

    def test_missing_px4_fails_before_importing_kit(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'extensions/pegasus.simulator/config/extension.toml'
            config.parent.mkdir(parents=True)
            config.touch()
            with self.assertRaises(SystemExit) as error:
                launcher.parse_arguments(['--pegasus-dir', directory,
                                          '--px4-dir', str(root / 'missing')])
            self.assertEqual(error.exception.code, 2)
            args = launcher.parse_arguments(['--pegasus-dir', directory, '--no-px4'])
            self.assertTrue(args.no_px4)

    def run_mocked_app(self, *, fail_extension=False, scene_error=None):
        app = MagicMock()
        app.is_running.return_value = True
        manager = MagicMock()
        manager.is_extension_enabled.return_value = not fail_extension
        timeline = MagicMock()
        isaacsim = types.ModuleType('isaacsim')
        isaacsim.SimulationApp = MagicMock(return_value=app)
        omni = types.ModuleType('omni')
        omni.kit = types.ModuleType('omni.kit')
        omni.kit.app = types.ModuleType('omni.kit.app')
        omni.kit.app.get_app = MagicMock()
        omni.kit.app.get_app.return_value.get_extension_manager.return_value = manager
        omni.timeline = types.ModuleType('omni.timeline')
        omni.timeline.get_timeline_interface = MagicMock(return_value=timeline)
        modules = {'isaacsim': isaacsim, 'omni': omni, 'omni.kit': omni.kit,
                   'omni.kit.app': omni.kit.app, 'omni.timeline': omni.timeline}
        args = types.SimpleNamespace(
            no_px4=True, no_px4_autolaunch=False, pegasus_dir=Path('/fake/PegasusSimulator'),
            headless=True, no_camera=False, no_lidar=False, no_ros2=False, max_steps=2,
        )
        follow = MagicMock()
        with patch.dict(sys.modules, modules), \
             patch.object(launcher, 'parse_arguments', return_value=args), \
             patch.object(launcher, 'build_scene', side_effect=scene_error,
                          return_value=(MagicMock(), follow)) as scene:
            if fail_extension or scene_error:
                with self.assertRaises(RuntimeError):
                    launcher.main(following=True)
            else:
                launcher.main(following=True)
        return app, manager, timeline, isaacsim.SimulationApp, scene, follow

    def test_bounded_loop_and_stop_callbacks_before_close(self):
        app, manager, timeline, constructor, scene, follow = self.run_mocked_app()
        self.assertEqual(follow.update.call_count, 2)
        timeline.play.assert_called_once()
        timeline.stop.assert_called_once()
        app.close.assert_called_once()
        # Bootstrap update, two loop updates and three stop-callback updates.
        self.assertEqual(app.update.call_count, 6)
        self.assertEqual(app.mock_calls[-1], unittest.mock.call.close())
        self.assertEqual(constructor.call_args.args[0]['extra_args'],
                         ['--ext-folder', '/fake/PegasusSimulator/extensions'])
        self.assertIn(unittest.mock.call('isaacsim.sensors.experimental.rtx', True),
                      manager.set_extension_enabled_immediate.call_args_list)
        self.assertTrue(scene.call_args.kwargs['following'])

    def test_failed_extension_closes_kit_without_building_scene(self):
        app, manager, timeline, constructor, scene, follow = self.run_mocked_app(
            fail_extension=True)
        scene.assert_not_called()
        timeline.play.assert_not_called()
        app.close.assert_called_once()

    def test_scene_failure_stops_timeline_and_closes_kit(self):
        app, manager, timeline, constructor, scene, follow = self.run_mocked_app(
            scene_error=RuntimeError('missing world'))
        timeline.play.assert_not_called()
        timeline.stop.assert_called_once()
        app.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
