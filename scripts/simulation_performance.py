"""Measure actual simulation progress against wall time without importing Kit."""
from time import perf_counter


class SimulationPerformance:
    def __init__(self, counters, *, interval=5.0, clock=perf_counter, report=print):
        self.counters = counters
        self.interval = interval
        self.clock = clock
        self.report = report
        self.wall = clock()
        self.sim, self.physics_steps = counters()
        self.updates = 0

    def sample(self):
        self.updates += 1
        wall = self.clock()
        elapsed = wall - self.wall
        if elapsed < self.interval:
            return
        sim, steps = self.counters()
        sim_elapsed = sim - self.sim
        physics_steps = steps - self.physics_steps
        # A timeline reset invalidates the previous measurement window.
        if elapsed > 0 and sim_elapsed >= 0 and physics_steps >= 0:
            self.report(
                f'SIM_PERF rtf={sim_elapsed / elapsed:.3f} '
                f'physics_hz={physics_steps / elapsed:.1f} '
                f'app_hz={self.updates / elapsed:.1f} '
                f'physics_steps_per_update={physics_steps / self.updates:.2f}', flush=True)
        self.wall, self.sim, self.physics_steps = wall, sim, steps
        self.updates = 0
