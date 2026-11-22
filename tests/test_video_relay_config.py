"""Camera discovery, stable UDP allocation, launch forwarding and console warnings."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from video_relay_config import video_relays
from world_config import load_world


def camera(name):
    return {'model_type': 'camera', 'model_name': name}


class RelayConfigTests(unittest.TestCase):
    def world(self, drones, **settings):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'world.yaml'
            path.write_text(yaml.safe_dump(dict(drones=drones, **settings)))
            return load_world(path)

    def test_names_and_nested_camera_order_determine_ports_not_list_order(self):
        world = self.world([
            {'model_name': 'drone2', 'instance': 0, 'payload': [camera('down')]},
            {'model_name': 'drone0', 'instance': 2, 'payload': [camera('front'),
                {'model_type': 'gimbal_speed', 'model_name': 'gimbal', 'payload': camera('zoom')}]},
            {'model_name': 'drone1', 'instance': 1, 'sensor_namespace': 'sensors/one',
             'payload': [camera('rear')]},
        ])
        routes = video_relays(world, '192.168.1.20')
        self.assertEqual([r['port'] for r in routes], [5620, 5600, 5601, 5610])
        self.assertEqual([r['camera'] for r in routes], ['down', 'front', 'zoom', 'rear'])
        self.assertEqual(routes[-1]['topic'], '/sensors/one/rear/color/image_raw')
        self.assertTrue(all(r['host'] == '192.168.1.20' for r in routes))

    def test_ten_cameras_and_arbitrary_drone_names(self):
        world = self.world([{'model_name': 'drone1', 'payload': [camera(f'cam{i}') for i in range(10)]}])
        self.assertEqual([r['port'] for r in video_relays(world)], list(range(5610, 5620)))
        world = self.world([{'model_name': 'leader', 'instance': 4, 'payload': [camera('front')]}])
        self.assertEqual(video_relays(world)[0]['port'], 5640)

    def test_invalid_or_ambiguous_allocations_fail(self):
        invalid = [
            [{'model_name': 'drone0', 'payload': [camera(f'cam{i}') for i in range(11)]}],
            [{'model_name': 'drone0', 'payload': [camera('front'), camera('front')]}],
            [{'model_name': 'drone0', 'payload': [camera('front')]},
             {'model_name': 'drone00', 'payload': [camera('rear')]}],
            [{'model_name': 'drone9999', 'payload': [camera('front')]}],
            [{'model_name': 'drone0', 'payload': [camera('invalid/name')]}],
        ]
        for drones in invalid:
            with self.subTest(drones=drones), self.assertRaises(ValueError):
                video_relays(self.world(drones))
        with self.assertRaisesRegex(ValueError, 'ros2: true'):
            video_relays(self.world([{'model_name': 'a', 'payload': [camera('front')]}], ros2=False))

    def test_launcher_defaults_off_then_warns_and_launches_only_selected_drone(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            world_path = root / 'world.yaml'
            world_path.write_text(yaml.safe_dump({'drones': [
                {'model_name': 'drone0', 'payload': [camera('front')]},
                {'model_name': 'drone2', 'payload': [camera('rear'), camera('down')]},
            ]}))
            log = root / 'commands.jsonl'
            tmuxinator = root / 'tmuxinator'
            tmuxinator.write_text('#!/usr/bin/env python3\nimport json,sys,os\n'
                                  'with open(os.environ["COMMAND_LOG"], "a") as f: f.write(json.dumps(sys.argv[1:])+"\\n")\n')
            ros2 = root / 'ros2'
            ros2.write_text('#!/bin/sh\nexit 1\n')
            tmuxinator.chmod(0o755)
            ros2.chmod(0o755)
            env = dict(os.environ, PATH=str(root)+':'+os.environ['PATH'], COMMAND_LOG=str(log), TMUX='')
            base = ['bash', str(ROOT / 'launch_as2.bash'), '--config', str(world_path),
                    '-n', 'drone2', '-s', '--no-attach', '--no-log-follow']
            result = subprocess.run(base, env=env, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            command = json.loads(log.read_text())
            self.assertIn('video_relay=false', command)
            self.assertNotIn('[video-relay]', result.stderr)
            log.write_text('')
            result = subprocess.run(base + ['--video-relay'], env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 2)
            self.assertIn('Build project_pegasus_isaac_sim', result.stderr)
            self.assertEqual(log.read_text(), '')
            ros2.write_text('#!/bin/sh\nexit 0\n')
            result = subprocess.run(base + ['--video-relay', '--video-relay-host', '192.168.1.20'],
                                    env=env, capture_output=True, text=True, timeout=15)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('[WARN] [video-relay] drone2/rear:', result.stderr)
            self.assertIn('udp://192.168.1.20:5620', result.stderr)
            self.assertIn('udp://192.168.1.20:5621', result.stderr)
            self.assertNotIn('[video-relay] drone0/', result.stderr)
            command = json.loads(log.read_text())
            self.assertIn('video_relay=true', command)
            plan_file = next(item.split('=', 1)[1] for item in command if item.startswith('video_relay_config_file='))
            routes = json.loads(Path(plan_file).read_text())
            self.assertEqual([r['port'] for r in routes], [5620, 5621])
            if shutil.which('ruby'):
                settings = dict(item.split('=', 1) for item in command if '=' in item)
                ruby = ('require "erb"; require "json"; @settings=JSON.parse(ARGV[1]); '
                        'puts ERB.new(File.read(ARGV[0]), trim_mode: "-").result(binding)')
                rendered = subprocess.run(['ruby', '-e', ruby, str(ROOT / 'tmuxinator/aerostack2.yaml'),
                                          json.dumps(settings)], capture_output=True, text=True, check=True).stdout
                document = yaml.safe_load(rendered)
                window = next(w['video_relay'] for w in document['windows'] if 'video_relay' in w)
                self.assertEqual(len(window['panes']), 2)
                self.assertIn('ros2 run project_pegasus_isaac_sim stream_video_udp.py', window['panes'][0])
                self.assertIn('--topic /drone2/rear/color/image_raw', window['panes'][0])
                self.assertIn('--port 5620', window['panes'][0])
                self.assertIn('__node:=video_relay_rear', window['panes'][0])
                self.assertIn('__ns:=/drone2', window['panes'][0])

    def test_no_cameras_prints_warning_and_writes_empty_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            config = root / 'world.yaml'
            config.write_text('drones: [{model_name: drone0}]\n')
            output = root / 'relays.json'
            result = subprocess.run([sys.executable, str(ROOT / 'scripts/video_relay_config.py'),
                '--config', str(config), '--drone', 'drone0', '--output', str(output)],
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(json.loads(output.read_text()), [])
            self.assertIn('no camera payloads; no relays started', result.stderr)
