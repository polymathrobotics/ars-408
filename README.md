# ars-408
Continental ARS 408 Driver for ROS2

# Configuring Filter Settings
The driver provides a convenient ros service interface to configure various filter parameters. The filter can be configured with the following command

```
ros2 service call /set_filter radar_conti_ars408_msgs/srv/SetFilter "type: <type>
index: <index>
sensor_id: <sensor_id>
min_value: <min_value>
max_value: <max_value>"
```

The `type` parameter should be set based upon the setting of the ARS408:
- 0: Cluster Mode
- 1: Object Mode

The `index` parameter specifies the property of the filter to be changed:
- 0: Number of objects
- 1: Distance
- 2: Azimuth
- 3: Relative oncoming velocity
- 4: Relative departing velocity
- 5: Radar cross section
- 6: Lifetime
- 7: Size
- 8: Probability of existence
- 9: Lateral distance
- 10: Longtudinal distance
- 11: Lateral velocty for right to left moving objects
- 12: Longitudial velocity for oncoming objects
- 13: Lateral velocity for left to right moving objects
- 14: Longitudial velocty for departing objects

The `sensor_id` parameter specifies the index of the sensor on the CAN bus

The `min_value` and `max_value` parameter specifies the minimum and maximum values of the radar respectively.

Further information about the filter configurations can be found in the ARS408 technical documentation
