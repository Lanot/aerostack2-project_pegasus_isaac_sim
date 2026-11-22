"""Apply Isaac Sim 6 sensor scheduling through the camera prim's public USD API."""


def set_camera_tick_rate(camera_prim, frequency):
    if not camera_prim or not camera_prim.ApplyAPI('OmniSensorAPI'):
        raise RuntimeError('Cannot apply OmniSensorAPI to the camera prim')
    attribute = camera_prim.GetAttribute('omni:sensor:tickRate')
    if not attribute or not attribute.Set(float(frequency)):
        raise RuntimeError('Cannot set camera omni:sensor:tickRate')
