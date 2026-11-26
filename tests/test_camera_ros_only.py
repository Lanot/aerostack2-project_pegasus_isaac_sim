"""ROS-only camera config and legacy acquisition lifecycle checks."""
import ast
from pathlib import Path
import sys
from unittest.mock import MagicMock

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from camera_timing import disable_python_camera_acquisition
from world_config import load_world


def write_world(tmp_path, camera_config, **extra):
    path=tmp_path/'world.yaml'
    path.write_text(yaml.safe_dump(dict(drones=[dict(model_name='drone0',payload=[
        dict(model_type='camera',model_name='front',config=camera_config)])],**extra)))
    return path


def test_ros_only_validation(tmp_path):
    path=write_world(tmp_path,dict(ros_only=True,frequency=25))
    world=load_world(path)
    assert world['drones'][0]['payload'][0]['config']['ros_only'] is True
    with pytest.raises(ValueError,match='must be a boolean'):
        load_world(write_world(tmp_path,dict(ros_only='true')))
    for options in (dict(ros2=False),dict(ros2_graphical_sensors=False)):
        with pytest.raises(ValueError,match='requires ros2'):
            load_world(write_world(tmp_path,dict(ros_only=True),**options))


@pytest.mark.parametrize('ros_only,depth',[(False,False),(False,True),(True,False),(True,True)])
def test_real_scheduled_camera_start_and_restart(ros_only,depth):
    tree=ast.parse((ROOT/'scripts/isaacsim_scene.py').read_text())
    cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='ScheduledCamera')
    class BaseCamera:
        def __init__(self,name,config):
            self._camera=MagicMock()
            self._depth=depth
            self._stage_prim_path='/World/drone0/body/front'
        def start(self):
            # Native wrapper initializes fresh acquisition/annotators every start.
            self._camera.initialize()
    namespace=dict(MonocularCamera=BaseCamera,
        disable_python_camera_acquisition=disable_python_camera_acquisition)
    exec(compile(ast.Module(body=[cls],type_ignores=[]),'scene','exec'),namespace)
    camera=namespace['ScheduledCamera']('front',dict(ros_only=ros_only))
    for _ in range(2):
        camera._camera.reset_mock()
        camera.start()
        camera._camera.initialize.assert_called_once()
        if ros_only:
            camera._camera.pause.assert_called_once()
            names=[c.args[0] for c in camera._camera.detach_annotator.call_args_list]
            assert names==(['rgb','distance_to_image_plane'] if depth else ['rgb'])
        else:
            camera._camera.pause.assert_not_called()
            camera._camera.detach_annotator.assert_not_called()
        # Keep the camera/render product object available to the native ROS helper.
        assert camera._camera is not None
