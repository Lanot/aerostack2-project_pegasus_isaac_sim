"""Apply Isaac Sim 6 sensor scheduling through the camera prim's public USD API."""


def set_camera_tick_rate(camera_prim, frequency):
    if not camera_prim or not camera_prim.ApplyAPI('OmniSensorAPI'):
        raise RuntimeError('Cannot apply OmniSensorAPI to the camera prim')
    attribute = camera_prim.GetAttribute('omni:sensor:tickRate')
    if not attribute or not attribute.Set(float(frequency)):
        raise RuntimeError('Cannot set camera omni:sensor:tickRate')


def disable_python_camera_acquisition(camera, *, depth=False):
    """Keep the render product for ROS helpers, without unused Python frame copies.

    The legacy wrapper's acquisition callback evaluates SDGPipeline synchronously
    and retrieves attached image annotators. Pegasus's ROS adapter needs only
    metadata and publishes through its own native helper writers.
    """
    camera.pause()
    camera.detach_annotator('rgb')
    if depth:
        camera.detach_annotator('distance_to_image_plane')
