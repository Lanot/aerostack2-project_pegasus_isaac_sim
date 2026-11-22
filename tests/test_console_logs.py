"""Check simulator log capture and detached-launch console forwarding offline."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ConsoleLogTests(unittest.TestCase):
    def test_detached_launcher_replays_startup_logs_and_can_return_immediately(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            tmuxinator = root / 'tmuxinator'
            tmuxinator.write_text('''#!/usr/bin/env python3
import pathlib, sys
settings = dict(arg.split('=', 1) for arg in sys.argv[1:] if '=' in arg)
if settings.get('base_launch') == 'true':
    pathlib.Path(settings['simulation_log_file']).write_text('simulator startup\\nPegasus startup\\n')
''')
            tail = root / 'tail'
            tail.write_text('#!/bin/bash\nprintf "VIEWER_STARTED\\n"\ncat "${@: -1}"\n')
            for binary in (tail, tmuxinator):
                binary.chmod(0o755)
            env = dict(os.environ, PATH=directory + ':' + os.environ['PATH'], TMUX='')
            for options, follows in (([], True), (['--no-log-follow'], False), (['-s'], False)):
                result = subprocess.run(['bash', str(ROOT / 'launch_as2.bash'), '-m',
                    '--no-attach', *options], env=env, text=True, capture_output=True, timeout=15)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual('VIEWER_STARTED' in result.stdout, follows)
                if follows:
                    self.assertIn('simulator startup', result.stdout)
                    self.assertIn('Pegasus startup', result.stdout)

    def test_template_tees_both_streams_and_preserves_failure(self):
        if not shutil.which('ruby'):
            self.skipTest('Ruby is required for the tmuxinator ERB template')
        with tempfile.TemporaryDirectory(prefix='as2 log ') as directory:
            root = Path(directory)
            log = root / 'simulator.log'
            (root / 'launch_isaacsim.bash').write_text(
                'echo "Isaac stdout"\necho "Pegasus stderr" >&2\n'
                'echo "UNBUFFERED=$PYTHONUNBUFFERED"\nexit 7\n')
            settings = dict(base_launch='true', simulation_config_file='config/world.yaml',
                            simulation_log_file=str(log), drone_namespace='drone0')
            program = ('require "erb"; require "json"; @settings=JSON.parse(ARGV[1]); '
                       'puts ERB.new(File.read(ARGV[0]), trim_mode: "-").result(binding)')
            rendered = subprocess.run(['ruby', '-e', program,
                str(ROOT / 'tmuxinator/aerostack2.yaml'), json.dumps(settings)],
                capture_output=True, text=True, check=True).stdout
            line = next(line for line in rendered.splitlines() if 'bash launch_isaacsim.bash' in line)
            command = line.strip().removeprefix('- ')
            result = subprocess.run(['bash', '-c', command], cwd=root, capture_output=True, text=True)
            self.assertEqual(result.returncode, 7)
            for message in ('Isaac stdout', 'Pegasus stderr', 'UNBUFFERED=1'):
                self.assertIn(message, result.stdout)
                self.assertIn(message, log.read_text())
