"""Offline validation of world parsing, routing, and swarm launcher commands."""
from pathlib import Path
import json
import os
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from world_config import load_world, platform_config


class WorldTests(unittest.TestCase):
    def write_world(self, directory, drones, **extra):
        path = Path(directory) / 'world.yaml'
        path.write_text(yaml.safe_dump(dict(world_name='empty', drones=drones, **extra)))
        return path

    def test_actual_gazebo_style_configs(self):
        single = load_world(ROOT / 'config/world.yaml')
        swarm = load_world(ROOT / 'config/world_swarm.yaml')
        self.assertEqual(single['drones'][0]['model_name'], 'drone0')
        self.assertEqual(single['drones'][0]['xyz'], [0., 0., 0.3])
        self.assertEqual([d['instance'] for d in swarm['drones']], [0, 1, 2])
        for drone in swarm['drones']:
            params = platform_config(swarm, drone['model_name'], ROOT / 'config/config.yaml')['/**']['ros__parameters']
            self.assertEqual(params['fmu_prefix'], '/' + drone['model_name'])
            self.assertEqual(params['target_system_id'], drone['instance'] + 1)
            self.assertTrue(params['use_sim_time'])

    def test_arbitrary_names_and_explicit_instance(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_world(directory, [{'model_name': 'leader', 'instance': 4}])
            world = load_world(path)
            params = platform_config(world, 'leader', ROOT / 'config/config.yaml')['/**']['ros__parameters']
            self.assertEqual(params['target_system_id'], 5)
            self.assertEqual(params['fmu_prefix'], '/leader')

    def test_duplicate_names_and_instances_and_invalid_pose_rejected(self):
        invalid = [
            [{'model_name': 'a'}, {'model_name': 'a'}],
            [{'model_name': 'a', 'instance': 0}, {'model_name': 'b', 'instance': 0}],
            [{'model_name': 'a', 'xyz': [float('nan'), 0, 1]}],
            [{'model_name': 'a', 'rpy': [0, 1]}],
            [{'model_name': '../unsafe'}],
            [],
        ]
        with tempfile.TemporaryDirectory() as directory:
            for drones in invalid:
                with self.subTest(drones=drones), self.assertRaises(ValueError):
                    load_world(self.write_world(directory, drones))

    def test_relative_usd_and_origin(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory) / 'scene.usda').touch()
            path = self.write_world(directory, [{'model_name': 'a'}], world_usd='scene.usda',
                                    origin={'latitude': 50, 'longitude': 30, 'altitude': 100})
            world = load_world(path)
            self.assertEqual(world['world_usd'], str(Path(directory) / 'scene.usda'))
            self.assertEqual(world['origin']['latitude'], 50)

    def test_scenario_cli_options_are_rejected(self):
        for options in (['--position', '1', '2', '3'], ['--no-px4'],
                        ['--no-camera'], ['--world-usd', 'test.usda'],
                        ['--vehicle-id', '2']):
            with self.subTest(options=options):
                result = subprocess.run([sys.executable,
                    str(ROOT / 'scripts/isaac_sim_run.py'),
                    '--print-config', *options], capture_output=True, text=True)
                self.assertEqual(result.returncode, 2)
                self.assertIn('unrecognized arguments', result.stderr)

    def test_yaml_controls_backends_camera_and_sensor_extensions(self):
        from isaacsim_app import parse_arguments
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_world(directory, [{
                'model_name': 'leader', 'instance': 4, 'xyz': [3, 0, .07],
                'payload': [{'model_type': 'lidar',
                             'config': {'sensor_configuration': 'Example_Rotary'}}],
            }], follow_camera=True, follow_drone='leader', ros2=False,
                px4={'enabled': False, 'autolaunch': False, 'model': 'custom'})
            args = parse_arguments(['--config', str(path), '--print-config'])
            self.assertTrue(args.follow_camera)
            self.assertEqual(args.follow_drone, 'leader')
            self.assertTrue(args.no_px4)
            self.assertTrue(args.no_px4_autolaunch)
            self.assertEqual(args.px4_model, 'custom')
            self.assertTrue(args.no_ros2)
            self.assertTrue(args.no_camera)
            self.assertFalse(args.no_lidar)
            self.assertEqual(args.world['drones'][0]['xyz'], [3., 0., .07])

    def test_lockstep_defaults_to_true_and_can_be_disabled(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_world(directory, [{'model_name': 'a'}])
            self.assertTrue(load_world(path)['px4']['enable_lockstep'])
            path = self.write_world(directory, [{'model_name': 'a'}],
                                    px4={'enabled': True, 'enable_lockstep': False})
            world = load_world(path)
            self.assertTrue(world['px4']['enabled'])
            self.assertFalse(world['px4']['enable_lockstep'])

    def test_camera_cli_overrides_yaml_and_prints_effective_config(self):
        from isaacsim_app import parse_arguments
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_world(directory, [{'model_name': 'a'}, {'model_name': 'b'}],
                                    follow_camera=True, follow_drone='a')
            base = ['--config', str(path), '--print-config']
            self.assertTrue(parse_arguments(base).follow_camera)
            args = parse_arguments(base + ['--no-follow-camera', '--follow-drone', 'b'])
            self.assertFalse(args.follow_camera)
            self.assertFalse(args.world['follow_camera'])
            self.assertEqual(args.world['follow_drone'], 'b')
            path = self.write_world(directory, [{'model_name': 'a'}], follow_camera=False)
            self.assertTrue(parse_arguments(base + ['--follow-camera']).follow_camera)
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/isaacsim_app.py'),
                                     *base, '--follow-drone', 'missing'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('must match a configured model_name', result.stderr)

    def test_invalid_backend_and_camera_settings_fail_before_kit(self):
        invalid = [{'px4': {'enable_lockstep': 'false'}}, {'px4': False}, {'px4': {'enabled': 'false'}},
                   {'px4': {'autolaunch': 1}}, {'px4': {'model': ''}},
                   {'ros2': 'false'}, {'follow_camera': 'yes'},
                   {'follow_drone': []}, {'follow_drone': 'missing'},
                   {'environment': False}]
        with tempfile.TemporaryDirectory() as directory:
            for options in invalid:
                with self.subTest(options=options), self.assertRaises(ValueError):
                    load_world(self.write_world(directory, [{'model_name': 'a'}], **options))

    def test_camera_override_is_forwarded_only_when_explicit(self):
        import shutil
        if not shutil.which('ruby'):
            self.skipTest('Ruby is required to render tmuxinator ERB')
        program = ('require "erb"; require "json"; '
                   '@settings=JSON.parse(ARGV[1]); '
                   'puts ERB.new(File.read(ARGV[0]), trim_mode: "-").result(binding)')
        for override, expected in (('', None), ('true', '--follow-camera'),
                                   ('false', '--no-follow-camera')):
            settings = dict(headless='true' if override == 'true' else 'false',
                            base_launch='true', drone_namespace='drone0',
                            follow_camera=override, follow_drone='drone0',
                            simulation_config_file='config/world.yaml')
            result = subprocess.run(['ruby', '-e', program,
                str(ROOT / 'tmuxinator/aerostack2.yaml'), json.dumps(settings)],
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            command = next(line for line in result.stdout.splitlines()
                           if 'bash launch_isaacsim.bash' in line)
            if expected:
                self.assertIn(expected, command)
            else:
                self.assertNotIn('--follow-camera', command)
                self.assertNotIn('--no-follow-camera', command)
            self.assertIn('--follow-drone drone0', command)
            self.assertEqual('--headless' in command, override == 'true')

    def test_swarm_launcher_one_sim_one_agent_and_correct_routing(self):
        with tempfile.TemporaryDirectory() as directory:
            binary = Path(directory) / 'tmuxinator'
            log = Path(directory) / 'commands.jsonl'
            binary.write_text('#!/usr/bin/env python3\nimport os,json,sys\n'
                              'with open(os.environ["COMMAND_LOG"],"a") as f: f.write(json.dumps(sys.argv[1:])+"\\n")\n')
            binary.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'],
                       COMMAND_LOG=str(log), TMUX='')
            result = subprocess.run(['bash', str(ROOT / 'launch_as2.bash'), '-m', '-x', '--no-attach', '--no-log-follow'],
                                    env=env, capture_output=True, text=True, cwd='/tmp')
            self.assertEqual(result.returncode, 0, result.stderr)
            commands = [json.loads(line) for line in log.read_text().splitlines()]
            self.assertEqual(len(commands), 3)
            self.assertEqual(sum('base_launch=true' in c for c in commands), 1)
            self.assertEqual(sum('micro_xrce_agent=true' in c for c in commands), 1)
            for index, command in enumerate(commands):
                value = next(c.split('=', 1)[1] for c in command if c.startswith('platform_config_file='))
                params = yaml.safe_load(Path(value).read_text())['/**']['ros__parameters']
                self.assertEqual(params['target_system_id'], index + 1)
            self.assertIn('follow_camera=', commands[0])
            for option, value in (('--follow-camera', 'true'), ('--no-follow-camera', 'false')):
                log.write_text('')
                result = subprocess.run(['bash', str(ROOT / 'launch_as2.bash'), '-m',
                    '--no-attach', '--no-log-follow', '--headless', option, '--follow-drone', 'drone2'],
                    env=env, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stderr)
                camera_commands = [json.loads(line) for line in log.read_text().splitlines()]
                self.assertIn('follow_camera=' + value, camera_commands[0])
                self.assertIn('follow_drone=drone2', camera_commands[0])
                self.assertIn('headless=true', camera_commands[0])
            log.write_text('')
            result = subprocess.run(['bash', str(ROOT / 'launch_ground_station.bash'), '-m', '-t', '--no-attach'],
                                    env=env, capture_output=True, text=True, cwd='/tmp')
            self.assertEqual(result.returncode, 0, result.stderr)
            command = json.loads(log.read_text())
            self.assertIn('drone_namespace=drone0,drone1,drone2', command)
            self.assertIn('keyboard_teleop=true', command)


if __name__ == '__main__':
    unittest.main()
