"""Exercise CLI forwarding, rendered simulator commands and tuning isolation."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]


def render(settings):
    if not shutil.which('ruby'):
        pytest.skip('Ruby is required to render tmuxinator ERB')
    program='require "erb"; require "json"; @settings=JSON.parse(ARGV[1]); puts ERB.new(File.read(ARGV[0]), trim_mode: "-").result(binding)'
    return subprocess.run(['ruby','-e',program,str(ROOT/'tmuxinator/aerostack2.yaml'),json.dumps(settings)],
        check=True,capture_output=True,text=True).stdout


def test_simulator_pane_sources_file_despite_stale_environment(tmp_path):
    scripts=tmp_path/'scripts';scripts.mkdir()
    shutil.copyfile(ROOT/'scripts/with_px4_env.bash',scripts/'with_px4_env.bash')
    tuning=tmp_path/'tuning $(touch unexpected).env'
    tuning.write_text('# tuning\nPX4_PARAM_COM_OF_LOSS_T=15\nexport PX4_PARAM_TEST_VALUE="two words"\n')
    log=tmp_path/'sim log.txt'
    (tmp_path/'launch_isaacsim.bash').write_text(
        'echo "LOSS=$PX4_PARAM_COM_OF_LOSS_T VALUE=$PX4_PARAM_TEST_VALUE"\n'
        'echo "UNBUFFERED=$PYTHONUNBUFFERED"\necho error >&2\nexit 7\n')
    output=render(dict(base_launch='true',px4_env_file=str(tuning),simulation_log_file=str(log),
        simulation_config_file='config/world.yaml',drone_namespace='drone0'))
    line=next(line for line in output.splitlines() if 'bash launch_isaacsim.bash' in line)
    result=subprocess.run(['bash','-c',line.strip().removeprefix('- ')],cwd=tmp_path,
        env=dict(os.environ,PX4_PARAM_COM_OF_LOSS_T='99'),text=True,capture_output=True)
    assert result.returncode==7
    assert 'LOSS=15 VALUE=two words' in result.stdout
    assert 'UNBUFFERED=1' in result.stdout
    assert 'error' in log.read_text()
    assert not (tmp_path/'unexpected').exists()
    # Only the simulator pane gets the wrapper; other AS2 commands are unaffected.
    assert output.count('with_px4_env.bash')==1
    output=render(dict(base_launch='false',px4_env_file=str(tuning),drone_namespace='drone1'))
    assert 'with_px4_env.bash' not in output


@pytest.mark.parametrize('flag',['--px4-env-file','--px4_env_file='])
def test_launcher_forwards_absolute_file_to_each_session(tmp_path,flag):
    capture=tmp_path/'calls.jsonl'
    binary=tmp_path/'tmuxinator'
    binary.write_text('#!/usr/bin/env python3\nimport json,os,sys\nwith open(os.environ["CAPTURE"],"a") as f: f.write(json.dumps(sys.argv[1:])+"\\n")\n')
    binary.chmod(0o755)
    tuning=tmp_path/'px4 tuning.env';tuning.write_text('PX4_PARAM_COM_OF_LOSS_T=15\n')
    # A relative path is resolved after launch_as2 changes into the project directory.
    relative=os.path.relpath(tuning,ROOT)
    option=[flag+relative] if flag.endswith('=') else [flag,relative]
    result=subprocess.run(['bash',str(ROOT/'launch_as2.bash'),'-m','--no-attach',
        '--no-log-follow',*option],cwd=tmp_path,env=dict(os.environ,TMUX='',CAPTURE=str(capture),
        PATH=str(tmp_path)+':'+os.environ['PATH']),text=True,capture_output=True)
    assert result.returncode==0,result.stderr
    calls=[json.loads(line) for line in capture.read_text().splitlines()]
    assert len(calls)==3
    assert all('px4_env_file='+str(tuning.resolve()) in call for call in calls)


@pytest.mark.parametrize('mode',['missing','directory','syntax','empty','no-simulation'])
def test_invalid_option_fails_before_launch(tmp_path,mode):
    capture=tmp_path/'launched'
    binary=tmp_path/'tmuxinator';binary.write_text('#!/bin/bash\ntouch "$CAPTURE"\n');binary.chmod(0o755)
    tuning=tmp_path/'tuning.env'
    if mode=='directory': tuning.mkdir()
    elif mode=='syntax': tuning.write_text('if then\n')
    elif mode=='no-simulation': tuning.write_text('PX4_PARAM_COM_OF_LOSS_T=15\n')
    options=['--px4-env-file='] if mode=='empty' else ['--px4-env-file',str(tuning)]
    if mode=='no-simulation': options+=['-s']
    result=subprocess.run(['bash',str(ROOT/'launch_as2.bash'),'--no-attach',*options],
        env=dict(os.environ,PATH=str(tmp_path)+':'+os.environ['PATH'],CAPTURE=str(capture)),
        text=True,capture_output=True)
    assert result.returncode!=0
    assert not capture.exists()


def test_source_error_stops_child_launch(tmp_path):
    tuning=tmp_path/'fail.env';tuning.write_text('false\n')
    result=subprocess.run(['bash',str(ROOT/'scripts/with_px4_env.bash'),str(tuning),
        sys.executable,'-c','print("CHILD_STARTED")'],text=True,capture_output=True)
    assert result.returncode!=0
    assert 'CHILD_STARTED' not in result.stdout
