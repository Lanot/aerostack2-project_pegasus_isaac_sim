"""Profiling preserves callbacks, exceptions, bound methods and concurrent counts."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from callback_profiler import CallbackProfiler, profiled_vehicle_class, profile_backend


class CallbackProfileTests(unittest.TestCase):
    def test_nested_wall_times_are_reported_inclusively(self):
        now = [0.0]
        output = Mock()
        profiler = CallbackProfiler(clock=lambda: now[0], report=output)
        def child():
            now[0] += .002
            return 42
        def parent():
            value = profiler.call('child', child)
            now[0] += .001
            return value
        self.assertEqual(profiler.call('parent', parent), 42)
        self.assertAlmostEqual(profiler.timings['parent'][0], .003)
        self.assertAlmostEqual(profiler.timings['child'][0], .002)
        now[0] = 5.0
        profiler.sample()
        lines = [call.args[0] for call in output.call_args_list]
        self.assertIn('nested rows overlap', lines[0])
        self.assertIn('parent total_ms=3.0 calls=1', lines[1])
        self.assertIn('child total_ms=2.0 calls=1', lines[2])
        self.assertEqual(profiler.timings, {})

    def test_callback_registered_during_init_uses_profiled_override(self):
        class Vehicle:
            def __init__(self):
                self.callback = self.update
            def update(self, dt, context=None):
                return self.apply_force(dt * 2)
            def apply_force(self, value):
                return value + 1
        profiler = CallbackProfiler()
        vehicle = profiled_vehicle_class(Vehicle, profiler, 'drone2')()
        self.assertEqual(vehicle.callback(.5), 2)
        self.assertEqual(profiler.timings['drone2.update'][1], 1)
        self.assertEqual(profiler.timings['drone2.apply_force'][1], 1)
        self.assertIsInstance(vehicle, Vehicle)

    def test_backend_method_returns_and_exceptions_are_preserved(self):
        class Backend:
            def update(self, dt):
                if dt < 0:
                    raise ValueError('bad dt')
                return dt * 2
        backend = Backend()
        profiler = CallbackProfiler()
        profile_backend(backend, profiler, 'drone0')
        self.assertEqual(backend.update(2), 4)
        with self.assertRaisesRegex(ValueError, 'bad dt'):
            backend.update(-1)
        self.assertEqual(profiler.timings['drone0.Backend.update'][1], 2)

    def test_worker_thread_callbacks_are_counted(self):
        profiler = CallbackProfiler()
        callback = profiler.wrap('worker', lambda: 1)
        def run(_):
            return sum(callback() for _ in range(100))
        with ThreadPoolExecutor(max_workers=4) as executor:
            self.assertEqual(sum(executor.map(run, range(4))), 400)
        self.assertEqual(profiler.timings['worker'][1], 400)
