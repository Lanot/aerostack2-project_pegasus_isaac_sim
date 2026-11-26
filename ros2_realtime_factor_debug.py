#!/usr/bin/env python3
"""Print /clock simulation progress / monotonic wall time every five seconds."""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


class RTFMonitor:
    """Constant-memory measurement windows, including time spent paused."""

    def __init__(self, clock=time.monotonic, report=print):
        self.clock = clock
        self.report = report
        self.latest_sim = None
        self.baseline_sim = None
        self.baseline_wall = None
        self.clock_reset = False

    def callback(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.latest_sim is not None and sim < self.latest_sim:
            self.clock_reset = True
        self.latest_sim = sim
        if self.baseline_wall is None:
            self.baseline_sim = sim
            self.baseline_wall = self.clock()

    def print_measurement(self):
        now = self.clock()
        if self.latest_sim is None:
            self.report('No usable /clock data; check simulation playback and ROS_DOMAIN_ID.', flush=True)
            return
        if self.clock_reset:
            self.report('Simulation clock reset; starting a new measurement window.', flush=True)
            self.clock_reset = False
        else:
            elapsed = now - self.baseline_wall
            if elapsed > 0:
                rtf = (self.latest_sim - self.baseline_sim) / elapsed
                self.report(f'Real-time factor: {rtf:.3f}', flush=True)
        self.baseline_sim = self.latest_sim
        self.baseline_wall = now


def main():
    rclpy.init()
    node = None
    try:
        node = rclpy.create_node('check_simulation_speed')
        monitor = RTFMonitor()
        subscription = node.create_subscription(
            Clock, '/clock', monitor.callback, qos_profile_sensor_data)
        interval = 5.0
        deadline = time.monotonic() + interval
        while rclpy.ok():
            timeout = max(0.0, min(0.1, deadline - time.monotonic()))
            rclpy.spin_once(node, timeout_sec=timeout)
            now = time.monotonic()
            if now >= deadline:
                monitor.print_measurement()
                deadline = now + interval
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
