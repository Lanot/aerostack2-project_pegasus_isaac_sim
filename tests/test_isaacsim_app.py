"""Offline launcher regressions; GPU/sensor/flight checks are documented in README."""
import ast
import importlib.util
from pathlib import Path
import subprocess
import sys
import types
import unittest
from io import StringIO
from contextlib import redirect_stderr
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('isaacsim_app', ROOT / 'isaacsim_app.py')
launcher = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launcher)


class LauncherTests(unittest.TestCase):
    def test_launcher_import_statements_are_all_at_module_top(self):
        tree = ast.parse((ROOT / 'isaacsim_app.py').read_text())
        top_imports = {id(node) for node in tree.body if isinstance(node, (ast.Import, ast.ImportFrom))}
        all_imports = {id(node) for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))}
        self.assertEqual(all_imports, top_imports)

    def test_entrypoint_help_without_isaacsim(self):
        result = subprocess.run(
            [sys.executable, str(ROOT / 'isaac_sim_run.py'), '--help'],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('--config', result.stdout)
        self.assertIn('--follow-camera', result.stdout)
        self.assertIn('--headless', result.stdout)
        self.assertNotIn('--world-usd', result.stdout)

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
            world = root / 'world.yaml'
            world.write_text('px4: {enabled: false}\ndrones: [{model_name: drone0}]\n')
            args = launcher.parse_arguments(['--pegasus-dir', directory, '--config', str(world)])
            self.assertTrue(args.no_px4)

    def run_mocked_app(self, *, fail_extension=False, scene_error=None, headless=False, extra_extensions=False, profile=False):
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
        scene_module = types.ModuleType('isaacsim_scene')
        scene_module.simulation_counters = MagicMock(return_value=(0.0, 0))
        def verify_error_before_close(*, exit_code):
            if scene_error:
                self.assertEqual(exit_code, 1)
                self.assertIn('Simulation failed during building world and spawning vehicles', errors.getvalue())
                self.assertIn(str(scene_error), errors.getvalue())
                self.assertIn('Traceback', errors.getvalue())
            elif fail_extension:
                self.assertEqual(exit_code, 1)
                self.assertIn('enabling extensions', errors.getvalue())
            else:
                self.assertEqual(exit_code, 0)
        errors = StringIO()
        app.close.side_effect = verify_error_before_close
        follow = MagicMock()
        scene_module.build_scene = MagicMock(side_effect=scene_error, return_value=(MagicMock(), follow))
        carb = types.SimpleNamespace(settings=types.SimpleNamespace(get_settings=MagicMock(return_value=MagicMock())))
        modules = {'carb': carb, 'isaacsim_scene': scene_module, 'isaacsim': isaacsim, 'omni': omni, 'omni.kit': omni.kit,
                   'omni.kit.app': omni.kit.app, 'omni.timeline': omni.timeline}
        args = types.SimpleNamespace(
            no_px4=True, no_px4_autolaunch=False, pegasus_dir=Path('/fake/PegasusSimulator'),
            world={'simulation': {'profile': profile}, 'extensions': ['cesium.omniverse'] if extra_extensions else [],
                   'extension_paths': ['/extensions/cesium'] if extra_extensions else []},
            headless=headless, print_config=False, follow_camera=True, no_camera=False, no_lidar=False, no_ros2=False, max_steps=2,
        )
        def import_after_startup(name):
            if name != 'isaacsim':
                isaacsim.SimulationApp.assert_called_once()
            if name == 'isaacsim_scene':
                self.assertTrue(app.update.called)
                for extension in ('pegasus.simulator', 'isaacsim.ros2.bridge', *args.world['extensions']):
                    self.assertIn(unittest.mock.call(extension, True),
                                  manager.set_extension_enabled_immediate.call_args_list)
            return modules[name]

        with patch.dict(sys.modules, modules), \
             patch.object(launcher, 'import_module', side_effect=import_after_startup), \
             patch.object(launcher, 'parse_arguments', return_value=args), \
             redirect_stderr(errors):
            scene = scene_module.build_scene
            if fail_extension or scene_error:
                with self.assertRaises(RuntimeError):
                    launcher.main()
            else:
                launcher.main()
        return app, manager, timeline, isaacsim.SimulationApp, scene, follow

    def test_bounded_loop_and_stop_callbacks_before_close(self):
        app, manager, timeline, constructor, scene, follow = self.run_mocked_app()
        self.assertEqual(follow.update.call_count, 2)
        timeline.play.assert_called_once()
        timeline.stop.assert_called_once()
        app.close.assert_called_once()
        # Bootstrap update, two loop updates and three stop-callback updates.
        self.assertEqual(app.update.call_count, 6)
        self.assertEqual(app.mock_calls[-1], unittest.mock.call.close(exit_code=0))
        self.assertFalse(constructor.call_args.args[0]['headless'])
        self.assertEqual(constructor.call_args.args[0]['extra_args'],
                         ['--ext-folder', '/fake/PegasusSimulator/extensions'])
        self.assertIn(unittest.mock.call('isaacsim.sensors.experimental.rtx', True),
                      manager.set_extension_enabled_immediate.call_args_list)
        self.assertTrue(scene.call_args.kwargs['following'])

    def test_profile_is_available_before_vehicle_callbacks_are_registered(self):
        _, _, _, _, scene, _ = self.run_mocked_app(profile=True)
        self.assertIsInstance(scene.call_args.args[1].callback_profiler, launcher.CallbackProfiler)

    def test_headless_disables_viewport_updates(self):
        _, _, _, constructor, _, _ = self.run_mocked_app(headless=True)
        self.assertTrue(constructor.call_args.args[0]['headless'])
        self.assertTrue(constructor.call_args.args[0]['disable_viewport_updates'])

    def test_cesium_paths_and_schemas_precede_stage_loading(self):
        _, manager, _, constructor, scene, _ = self.run_mocked_app(extra_extensions=True)
        extra_args = constructor.call_args.args[0]['extra_args']
        self.assertEqual(extra_args[:4], ['--ext-folder', '/fake/PegasusSimulator/extensions',
                                         '--ext-folder', '/extensions/cesium'])
        self.assertIn('cesium.usd.plugins', extra_args)
        self.assertIn('--/app/useFabricSceneDelegate=true', extra_args)
        self.assertIn('--/exts/cesium.omniverse/showOnStartup=false', extra_args)
        self.assertIn(unittest.mock.call('cesium.omniverse', True),
                      manager.set_extension_enabled_immediate.call_args_list)
        scene.assert_called_once()

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
