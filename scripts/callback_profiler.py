"""Optional inclusive wall-time profiling for Python callbacks, without Kit imports."""
from functools import wraps
from threading import Lock
from time import perf_counter


class CallbackProfiler:
    def __init__(self, *, interval=5.0, clock=perf_counter, report=print):
        self.interval = interval
        self.clock = clock
        self.report = report
        self.window = clock()
        self.lock = Lock()
        self.timings = {}

    def call(self, label, function, *args, **kwargs):
        start = self.clock()
        try:
            return function(*args, **kwargs)
        finally:
            elapsed = self.clock() - start
            with self.lock:
                total, count, maximum = self.timings.get(label, (0.0, 0, 0.0))
                self.timings[label] = (total + elapsed, count + 1, max(maximum, elapsed))

    def wrap(self, label, function):
        @wraps(function)
        def measured(*args, **kwargs):
            return self.call(label, function, *args, **kwargs)
        return measured

    def sample(self):
        now = self.clock()
        if now - self.window < self.interval:
            return
        with self.lock:
            timings, self.timings = self.timings, {}
        self.window = now
        self.report('SIM_PROFILE inclusive wall times; nested rows overlap; top 12:', flush=True)
        for label, (total, count, maximum) in sorted(timings.items(), key=lambda row: row[1][0], reverse=True)[:12]:
            self.report(f'SIM_PROFILE {label} total_ms={total * 1000:.1f} calls={count} '
                        f'avg_ms={total * 1000 / count:.3f} max_ms={maximum * 1000:.3f}', flush=True)


def profiled_vehicle_class(base, profiler, name):
    """Wrap overrides before Vehicle.__init__ registers its bound callbacks."""
    methods = ('update', 'update_state', 'update_sensors', 'update_graphical_sensors',
               'update_sim_state', 'apply_force', 'apply_torque', 'handle_propeller_visual',
               'flush_direct_tensors')
    overrides = {method: profiler.wrap(f'{name}.{method}', getattr(base, method))
                 for method in methods if callable(getattr(base, method, None))}
    return type('Profiled' + base.__name__, (base,), overrides)


def profile_backend(backend, profiler, name):
    for method in ('update', 'update_sensor', 'update_state', 'update_graphical_sensor'):
        function = getattr(backend, method, None)
        if callable(function):
            label = f'{name}.{type(backend).__name__}.{method}'
            setattr(backend, method, profiler.wrap(label, function))
