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

The `min_value` and `max_value` parameter specifies the minimum and maximum values of the radar respectively in integer. All parameters except for longitudinal distance are 12-bit so max value would be 4095.The scaling of 12-bit number to actual SI units can be found in the ARS408 technical documentation

Further information about the filter configurations can be found in the ARS408 technical documentation

# Configuring Sensor ID
Ideally the sensors should be already configured prior to the use of the driver. With `can-utils`, one can configure the sensor with the following `cansend` command:

```
cansend can0 2<index of sensor to be changed>0#820000000<desired index>800000
```

If sensor 0 is to be changed to sensor 1, command would be as follows:

```
cansend can0 200#8200000001800000
```

# Configuring Sensor Power
The radar power may need to be changed depending on the application. For applications where the sensor needs to be in close proximity to obstacles the power should be lower. With `can-utils`, one can configure the sensor in the following way

For standard gain:
```
cansend can0 2<index of sensor to be changed>0#8400000000800000
```  

For -3dB gain
```
cansend can0 2<index of sensor to be changed>0#8400000020800000
```

For -6dB gain
```
cansend can0 2<index of sensor to be changed>0#8400000040800000
```

For -9dB gain
```
cansend can0 2<index of sensor to be changed>0#8400000060800000
```

If sensor 0 is to be changed to -6dB gain, command would be as follows:

```
cansend can0 200#8200000040800000
```
