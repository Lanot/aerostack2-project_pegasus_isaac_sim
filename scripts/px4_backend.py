"""Per-vehicle PX4 launch environment for a Pegasus swarm (import after Kit starts)."""
import os

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend
from pegasus.simulator.logic.backends.tools.px4_launch_tool import PX4LaunchTool


class NamespacedPX4Backend(PX4MavlinkBackend):
    def __init__(self, config, namespace):
        super().__init__(config)
        self.dds_namespace = namespace

    def start(self):
        if self._is_running:
            return
        if self.px4_autolaunch and self.px4_tool is None:
            # The upstream tool keeps os.environ by reference. Give each process
            # its own snapshot so drone2 cannot overwrite drone0's namespace.
            previous_model = os.environ.get('PX4_SIM_MODEL')
            try:
                tool = PX4LaunchTool(self.px4_dir, self._vehicle_id, self.px4_vehicle_model)
                tool.environment = dict(tool.environment)
            finally:
                if previous_model is None:
                    os.environ.pop('PX4_SIM_MODEL', None)
                else:
                    os.environ['PX4_SIM_MODEL'] = previous_model
            tool.environment['PX4_UXRCE_DDS_NS'] = self.dds_namespace
            self.px4_tool = tool
            try:
                tool.launch_px4()
            except Exception:
                self.px4_tool = None
                raise
        # With px4_tool populated, the parent starts communication without
        # launching a second process. Its stop() remains responsible for cleanup.
        super().start()
