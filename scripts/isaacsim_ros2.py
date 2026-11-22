"""Project-local sensor publication adapter for the installed Isaac Sim 6.x API.

Import only after SimulationApp and isaacsim.ros2.bridge are enabled.
"""
import omni.graph.core as og
import omni.replicator.core as rep
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend


class IsaacSim6ROS2Backend(ROS2Backend):
    """Keep Pegasus scheduling but let current helper nodes build sensor writers.

    The older backend reshapes CameraInfo lists as NumPy arrays and manually
    controls a legacy IsaacSimulationGate. The helper nodes own these details
    and derive calibration directly from the render product in Isaac Sim 6.
    """

    def __init__(self, vehicle_id, num_rotors=4, config=None):
        config = dict(config or {})
        self._exact_namespace = config.get('namespace_exact')
        super().__init__(vehicle_id=vehicle_id, num_rotors=num_rotors, config=config)

    def initialize_publishers(self, config):
        if self._exact_namespace is None:
            return super().initialize_publishers(config)

        namespace, vehicle_id = self._namespace, self._id
        try:
            # Parent constructs topic names by concatenating namespace and ID.
            # Keep the numeric node ID, but use the exact model_name for topics.
            self._namespace, self._id = self._exact_namespace, ''
            return super().initialize_publishers(config)
        finally:
            self._namespace, self._id = namespace, vehicle_id

    def _sensor_graph(self, data, name, render_product, helpers):
        keys = og.Controller.Keys
        nodes = [('Tick', 'omni.graph.action.OnPlaybackTick')]
        connections = []
        values = []

        for node, node_type, topic, output_type in helpers:
            nodes.append((node, 'isaacsim.ros2.bridge.' + node_type))
            connections.append(('Tick.outputs:tick', node + '.inputs:execIn'))

            values.extend([
                (node + '.inputs:renderProductPath', render_product),
                (node + '.inputs:nodeNamespace', getattr(self, '_exact_namespace', None) or self._namespace + str(self._id)),
                (node + '.inputs:frameId', name),
                (node + '.inputs:topicName', topic),
                (node + '.inputs:useSystemTime', False),
            ])

            if output_type is not None:
                values.append((node + '.inputs:type', output_type))

        return og.Controller.edit(
            {'graph_path': data['stage_prim_path'] + '/ROS2Publish',
             'evaluator_name': 'execution'},
            {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
        )

    def add_monocular_camera_writter(self, data):
        name = data['camera_name']
        helpers = [
            ('RGB', 'ROS2CameraHelper', name + '/color/image_raw', 'rgb'),
            ('Info', 'ROS2CameraInfoHelper', name + '/color/camera_info', None),
        ]

        if data.get('depth', False):
            helpers.append(('Depth', 'ROS2CameraHelper', name + '/depth', 'depth'))

        graph = self._sensor_graph(data, name, data['camera']._render_product_path, helpers)

        self.graphical_sensors_writers[name] = [graph]

    def add_lidar_writter(self, data):
        name = data['lidar_name']
        product = rep.create.render_product(data['stage_prim_path'], (1, 1), name=name)

        graph = self._sensor_graph(data, name, product.path, [
            ('Points', 'ROS2RtxLidarHelper', name + '/pointcloud', 'point_cloud'),
            ('Scan', 'ROS2RtxLidarHelper', name + '/laserscan', 'laser_scan'),
        ])

        # Retain the render product for the lifetime of the backend.
        self.graphical_sensors_writers[name] = [graph, product]
