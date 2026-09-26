"""Run with PYTHONPATH=exercises/appD/starter or .../solutions."""
from topics import sensor_topics
available=['/clock','/ground_truth/odometry','/tf','/scan','/imu/data_raw','/map']
assert sensor_topics(available)==['/scan','/imu/data_raw'],'Do not replay truth, map or a second clock'
assert sensor_topics(['/imu/data_raw','/scan'])==['/scan','/imu/data_raw']
try:
    sensor_topics(['/scan'])
except ValueError:
    pass
else:
    raise AssertionError('Missing IMU must be reported')
print('PASS: explicit sensor topics and missing-input check')
