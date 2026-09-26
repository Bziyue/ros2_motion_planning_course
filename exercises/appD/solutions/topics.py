"""Explicit input selection instead of recording every discovered topic."""
def sensor_topics(available):
    """Return the two course sensors, requiring both to be present."""
    required=['/scan','/imu/data_raw']
    if not set(required).issubset(available):
        raise ValueError('Both scan and raw IMU are required')
    return required
