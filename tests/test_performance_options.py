"""Offline checks for configuration forwarding and Pegasus tensor semantics."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from world_config import add_simulation_arguments, simulation_overrides, load_world
from simulation_settings import apply_simulation_settings
from pegasus_direct_tensors import direct_vehicle_class, LocalWrenches
from callback_profiler import CallbackProfiler, profiled_vehicle_class


def test_cli_overrides_and_explicit_false(tmp_path):
    path = tmp_path / 'world.yaml'
    path.write_text('drones: [{model_name: drone0}]\nsimulation: {profile: true, direct_tensors: true}\n')
    parser = argparse.ArgumentParser()
    add_simulation_arguments(parser)
    args = parser.parse_args(['--physics_device', 'cpu', '--physics-hz', '300',
        '--physics_threads', '0', '--render_quality', 'fast', '--no-profile', '--no-direct_tensors'])
    actual = load_world(path, simulation_overrides(args))['simulation']
    assert actual == dict(physics_device='cpu', physics_hz=300, render_hz=50,
        physics_threads=0, profile=False, render_quality='fast', direct_tensors=False)
    assert simulation_overrides(parser.parse_args([])) == {}


@pytest.mark.parametrize('key,value', [('physics_device','gpu'), ('physics_hz',0),
    ('physics_threads',-2), ('physics_threads',True), ('direct_tensors','true'),
    ('profile',1), ('render_quality','unknown')])
def test_reject_invalid_settings(tmp_path, key, value):
    path = tmp_path / 'world.yaml'
    path.write_text(yaml.safe_dump({'drones':[{'model_name':'drone0'}], 'simulation':{key:value}}))
    with pytest.raises(ValueError):
        load_world(path)


def test_thread_zero_and_render_overrides():
    settings = MagicMock()
    apply_simulation_settings(settings, {'physics_threads':-1})
    settings.set_int.assert_not_called()
    settings.set_bool.assert_not_called()
    apply_simulation_settings(settings, {'physics_threads':0, 'render_quality':'fast'})
    settings.set_int.assert_called_once_with('/persistent/physics/numThreads', 0)
    assert settings.set_bool.call_count == 3


def test_full_as2_launcher_forwards_effective_world(tmp_path):
    capture = tmp_path / 'launches.txt'
    binary = tmp_path / 'tmuxinator'
    binary.write_text('#!/usr/bin/env python3\nimport os,sys\nwith open(os.environ["CAPTURE"],"a") as f: f.write(" ".join(sys.argv[1:])+"\\n")\n')
    binary.chmod(0o755)
    env = dict(os.environ, PATH=str(tmp_path)+':'+os.environ['PATH'], CAPTURE=str(capture), TMUX='')
    result = subprocess.run(['bash', 'launch_as2.bash', '-m', '--no-attach', '--no-log-follow',
        '--physics_device', 'cpu', '--physics_hz=300', '--physics_threads', '0',
        '--render_quality', 'fast', '--profile', '--direct_tensors'],
        cwd=ROOT, env=env, text=True, capture_output=True, timeout=20)
    assert result.returncode == 0, result.stderr
    words = capture.read_text().split()
    paths = [word.split('=',1)[1] for word in words if word.startswith('simulation_config_file=')]
    assert paths
    world = yaml.safe_load(Path(paths[0]).read_text())
    assert world['simulation'] == dict(physics_device='cpu', physics_hz=300,
        render_hz=50, physics_threads=0, profile=True, render_quality='fast', direct_tensors=True)


class FakeIO:
    def __init__(self, prefix, device):
        self.body_parts = ['/rotor2','/body','/rotor0','/rotor3','/rotor1']
        self.apply_wrenches = MagicMock()
    def read_body_state(self):
        # A 90 degree yaw, world velocity X and world angular velocity Y.
        return np.array([1,2,3,0,0,np.sqrt(.5),np.sqrt(.5),2,0,0,0,3,0])


class BaseVehicle:
    def __init__(self):
        self._stage_prefix = '/World/drone0'
        self._thrusters = SimpleNamespace(_num_rotors=4, rot_dir=[1,-1,1,-1])
        self._state = SimpleNamespace(linear_velocity=np.zeros(3))
        self.original_force = MagicMock()
    def start(self):
        self._articulation = MagicMock()
    def stop(self):
        self._articulation = None
    def update_state(self, dt, context=None):
        pass
    def _get_rotor_dof_index(self, i):
        return i+2
    def apply_force(self, *args):
        self.original_force(*args)
    def apply_torque(self, *args):
        pass
    def handle_propeller_visual(self, *args):
        pass
    def update(self, dt, context=None):
        for i, force in enumerate([0,.05,.1,2]):
            self.apply_force([0,0,force], body_part='/rotor'+str(i))
            self.handle_propeller_visual(i,force)
        self.apply_force([2,0,0], pos=[0,1,0])
        self.apply_torque([0,0,4])


def test_direct_vehicle_frames_batches_callbacks_and_restart():
    cls = direct_vehicle_class(BaseVehicle, device='cpu', io_factory=FakeIO)
    cls = profiled_vehicle_class(cls, CallbackProfiler(), 'drone0')
    vehicle = cls()
    assert vehicle.update.__name__ == 'update'
    assert vehicle.update_state.__name__ == 'update_state'
    vehicle.start()
    io = vehicle._direct_io
    vehicle.update_state(.5)
    np.testing.assert_allclose(vehicle._state.linear_body_velocity, [0,-2,0], atol=1e-14)
    np.testing.assert_allclose(vehicle._state.angular_velocity, [3,0,0], atol=1e-14)
    np.testing.assert_allclose(vehicle._state.linear_acceleration, [4,0,0])
    vehicle.update(.5)
    io.apply_wrenches.assert_called_once()
    wrenches = io.apply_wrenches.call_args.args[0]
    np.testing.assert_allclose(wrenches[0,:,2], [.1,0,0,2,.05])
    np.testing.assert_allclose(wrenches[0,1], [2,0,0])
    np.testing.assert_allclose(wrenches[1,1], [0,0,2])  # 4 + [0,1,0] x [2,0,0]
    vehicle._articulation.set_dof_velocities.assert_called_once_with([[0,-5,100,-100]], dof_indices=[2,3,4,5])
    vehicle.apply_force([1,2,3])  # External calls retain immediate submission.
    vehicle.original_force.assert_called_once()
    vehicle.stop()
    assert vehicle._direct_io is None
    vehicle.start()
    assert vehicle._direct_io is not io


def test_local_force_moment_equivalence():
    batch = LocalWrenches(['/body','/rotor0'])
    force, pos, com = np.array([2,3,5]), np.array([.4,-.2,.1]), np.array([.1,.2,.3])
    batch.force(force, pos, '/rotor0')
    np.testing.assert_allclose(np.cross(pos-com,force),
        batch.data[1,1] + np.cross(-com,batch.data[0,1]), atol=1e-6)


def test_warp_io_native_order_and_buffer_reuse():
    wp = pytest.importorskip('warp')
    wp.config.kernel_cache_dir = '/tmp/as2-pegasus-warp-tests'
    from pegasus_tensor_io import WarpTensorIO
    prefix = '/World/drone0'
    paths = [prefix+part for part in ['/rotor1','/rotor0','/body','/rotor2','/rotor3']]
    transforms = np.arange(35,dtype=np.float32).reshape(5,7)
    velocities = np.arange(30,dtype=np.float32).reshape(5,6)
    view = SimpleNamespace(prim_paths=paths, get_transforms=lambda:wp.array(transforms,device='cpu'),
        get_velocities=lambda:wp.array(velocities,device='cpu'), apply_forces_and_torques_at_position=MagicMock())
    rigid = SimpleNamespace(_physics_rigid_body_view=view, is_physics_tensor_entity_valid=lambda:True)
    io = WarpTensorIO(prefix, device='cpu', rigid_prim_class=lambda **kwargs:rigid)
    state = io.read_body_state()
    np.testing.assert_array_equal(state, np.concatenate([transforms[2],velocities[2]]))
    state[:] = -1
    assert io.read_body_state()[0] == transforms[2,0]
    buffer = io.packed_device
    for value in (2,4):
        data = np.full((2,5,3),value,dtype=np.float32)
        io.apply_wrenches(data)
        f,t,p,indices,global_frame = view.apply_forces_and_torques_at_position.call_args.args
        np.testing.assert_array_equal(f.numpy(), data[0])
        np.testing.assert_array_equal(t.numpy(), data[1])
        np.testing.assert_array_equal(p.numpy(), np.zeros((5,3)))
        np.testing.assert_array_equal(indices.numpy(), np.arange(5))
        assert global_frame is False
        assert io.packed_device is buffer
