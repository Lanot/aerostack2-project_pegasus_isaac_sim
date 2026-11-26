"""Validate world settings, native Pegasus scheduling and scene propagation."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from sensor_rates import configure_sensor_rates
from world_config import load_world


@pytest.mark.parametrize('rates', [None, [], {'unknown': 50}, {'imu': 0}, {'gps': -1},
    {'gps': True}, {'gps': '10'}, {'imu': 500}, {'gps': float('nan')}, {'gps': float('inf')}])
def test_invalid_rates_rejected(tmp_path, rates):
    path = tmp_path / 'world.yaml'
    path.write_text(yaml.safe_dump({'drones':[{'model_name':'drone0'}],
        'simulation':{'sensor_rates':rates}}))
    with pytest.raises(ValueError):
        load_world(path)


def test_omission_preserves_defaults_and_partial_settings(tmp_path):
    path = tmp_path / 'world.yaml'
    world = {'drones':[{'model_name':'drone0'}]}
    path.write_text(yaml.safe_dump(world))
    assert 'sensor_rates' not in load_world(path)['simulation']
    world['simulation']={'sensor_rates':{'gps':10}}
    path.write_text(yaml.safe_dump(world))
    assert load_world(path)['simulation']['sensor_rates'] == {'gps':10}
    imu=SimpleNamespace(sensor_type='IMU',update_rate=250,set_update_rate=MagicMock())
    gps=SimpleNamespace(sensor_type='GPS',update_rate=250,set_update_rate=MagicMock())
    configure_sensor_rates([imu,gps],{'gps':10},'drone0',report=MagicMock())
    imu.set_update_rate.assert_not_called()
    gps.set_update_rate.assert_called_once_with(10)


def test_installed_pegasus_sensor_scheduling():
    # Load the pure Sensor class without importing Pegasus's Kit initializers.
    source=Path('/home/tolik/sim/IsaacSim-Extensions/PegasusSimulator/extensions/pegasus.simulator/pegasus/simulator/logic/sensors/sensor.py')
    if not source.is_file():
        pytest.skip('Installed Pegasus source is unavailable')
    tree=ast.parse(source.read_text())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='Sensor')
    namespace={'State':object}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),namespace)
    Sensor=namespace['Sensor']
    class CountingSensor(Sensor):
        def __init__(self, name):
            super().__init__(name,250)
            self.samples=[]
        @Sensor.update_at_rate
        def update(self,state,dt):
            self.samples.append(dt)
            return dt
    sensors=[CountingSensor(name) for name in ('IMU','Magnetometer','Barometer','GPS')]
    configure_sensor_rates(sensors,{'imu':250,'magnetometer':50,'barometer':50,'gps':10},'drone0')
    for _ in range(2500):
        for sensor in sensors:
            sensor.update(None, .004)
    # Initial sampling happens immediately; subsequent periods follow the native scheduler.
    assert len(sensors[0].samples)==2500
    assert len(sensors[1].samples)==500
    assert len(sensors[2].samples)==500
    assert len(sensors[3].samples)==100
    assert sensors[3].samples[1] == pytest.approx(.1)


def test_scene_applies_rates_before_vehicle_construction():
    # Execute the actual scene function with API doubles, avoiding Kit startup.
    tree=ast.parse((ROOT/'scripts/isaacsim_scene.py').read_text())
    spawn=next(node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name=='spawn_vehicle')
    sensors=[]
    for name in ('IMU','GPS'):
        sensor=SimpleNamespace(sensor_type=name,update_rate=250)
        sensor.set_update_rate=MagicMock(side_effect=lambda rate,s=sensor:setattr(s,'update_rate',rate))
        sensors.append(sensor)
    config=SimpleNamespace(sensors=sensors)
    vehicle=MagicMock()
    namespace={'MultirotorConfig':lambda:config,'configure_sensor_rates':configure_sensor_rates,
        'Multirotor':vehicle,'ROBOTS':{'Iris':'iris.usd'},'Rotation':MagicMock()}
    exec(compile(ast.Module(body=[spawn],type_ignores=[]),'scene','exec'),namespace)
    args=SimpleNamespace(world={'simulation':{'sensor_rates':{'gps':10},'direct_tensors':False}},
        no_px4=True,no_ros2=True,no_camera=True,no_lidar=True)
    spec={'model_name':'drone0','instance':0,'_payloads':[],'xyz':[0,0,1],'rpy':[0,0,0],
          'pegasus_model':'Iris','model_type':'quadrotor_base'}
    namespace['spawn_vehicle'](MagicMock(),args,spec,False,MagicMock())
    assert sensors[0].update_rate==250
    assert sensors[1].update_rate==10
    assert vehicle.call_args.kwargs['config'] is config
