"""Optional physical-sensor sampling rates; no Isaac Sim imports."""
import math

SENSOR_TYPES = {
    'imu': 'IMU', 'magnetometer': 'Magnetometer',
    'barometer': 'Barometer', 'gps': 'GPS',
}


def validate_sensor_rates(rates, physics_hz):
    if not isinstance(rates, dict):
        raise ValueError('simulation.sensor_rates must be a mapping')
    unknown = set(rates) - SENSOR_TYPES.keys()
    if unknown:
        raise ValueError(f'Unknown simulation.sensor_rates keys: {list(map(str, unknown))}')
    for name, rate in rates.items():
        if (isinstance(rate, bool) or not isinstance(rate, (int, float))
                or not math.isfinite(rate) or not 0 < rate <= physics_hz):
            raise ValueError(f'simulation.sensor_rates.{name} must be a finite positive '
                             f'number no greater than physics_hz ({physics_hz})')


def configure_sensor_rates(sensors, rates, vehicle_name, report=print):
    """Apply explicit overrides before sensor initialization; retain other defaults."""
    present = {sensor.sensor_type: sensor for sensor in sensors}
    for name, rate in rates.items():
        sensor_type = SENSOR_TYPES[name]
        if sensor_type not in present:
            raise ValueError(f'{vehicle_name}: cannot configure missing sensor {sensor_type}')
        present[sensor_type].set_update_rate(rate)
    actual = ' '.join(f'{sensor.sensor_type}={sensor.update_rate:g}Hz' for sensor in sensors)
    report(f'SIM_SENSOR_RATES {vehicle_name} {actual}', flush=True)
