"""Measure physics progress rather than mistaking UI FPS for simulation speed."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from simulation_performance import SimulationPerformance


class PerformanceTests(unittest.TestCase):
    def test_reports_physics_progress_separately_from_app_updates(self):
        clock = Mock(return_value=10.0)
        counters = Mock(return_value=(0.0, 0))
        output = Mock()
        monitor = SimulationPerformance(counters, clock=clock, report=output)
        # 250 app ticks, five physics steps each, advance five simulation seconds.
        for i in range(1, 251):
            clock.return_value = 10.0 + i * .02
            counters.return_value = (i * .02, i * 5)
            monitor.sample()
        output.assert_called_once()
        message = output.call_args.args[0]
        self.assertIn('rtf=1.000', message)
        self.assertIn('physics_hz=250.0', message)
        self.assertIn('app_hz=50.0', message)
        self.assertIn('physics_steps_per_update=5.00', message)
        # Physics counters are queried once per reporting window, not every tick.
        self.assertEqual(counters.call_count, 2)

    def test_reset_restarts_measurement_and_stall_reports_zero_progress(self):
        clock = Mock(return_value=0.0)
        counters = Mock(return_value=(100.0, 25000))
        output = Mock()
        monitor = SimulationPerformance(counters, clock=clock, report=output)
        clock.return_value = 5.0
        counters.return_value = (0.0, 0)
        monitor.sample()
        output.assert_not_called()
        clock.return_value = 10.0
        monitor.sample()
        self.assertIn('rtf=0.000', output.call_args.args[0])
        self.assertIn('physics_hz=0.0', output.call_args.args[0])
