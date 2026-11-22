"""Verify public USD scheduling attributes and explicit schema failures offline."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from camera_timing import set_camera_tick_rate


class CameraTimingTests(unittest.TestCase):
    def test_authors_native_rate_on_camera_prim(self):
        prim = Mock()
        prim.ApplyAPI.return_value = True
        prim.GetAttribute.return_value.Set.return_value = True
        set_camera_tick_rate(prim, 10)
        prim.ApplyAPI.assert_called_once_with('OmniSensorAPI')
        prim.GetAttribute.assert_called_once_with('omni:sensor:tickRate')
        prim.GetAttribute.return_value.Set.assert_called_once_with(10.0)

    def test_failed_schema_or_attribute_is_not_silently_ignored(self):
        prim = Mock()
        prim.ApplyAPI.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'OmniSensorAPI'):
            set_camera_tick_rate(prim, 10)
        prim.ApplyAPI.return_value = True
        prim.GetAttribute.return_value.Set.return_value = False
        with self.assertRaisesRegex(RuntimeError, 'tickRate'):
            set_camera_tick_rate(prim, 10)
