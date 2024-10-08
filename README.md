# ars-408

Continental ARS 408 Driver for ROS2.

## Getting Started

### Quick Start

```
ros2 launch radar_conti_ars408 radar.launch.py
```

### Configuring Sensor ID

In the case where you have multiple Continental radars, you'll need to configure each sensor id. Each CAN frame id's second byte is encoded with the sensor id. 

With `can-utils`, one can configure the sensor with the following `cansend` command:

```
cansend can0 2<index of sensor to be changed>0#820000000<desired index>800000
```

If sensor 0 is to be changed to sensor 1, command would be as follows:

```
cansend can0 200#8200000001800000
```

### Configuring Radar Settings

There are ros params for setting the radar configuration that are set declaratively by the ROS2 configuration service (`/radar_conti_ars408/set_radar_configuration`). Unlike the filter configuration, which fires off filter settings on startup of the driver, the radar configurations shouldn't be overly used. From the Continental docs:

> It is important to note that the number of transmissions to the NVM should be kept to a minimum as this could reduce the service life of the memory.

Therefore, when you want to update the configuration, simply call the service with:
```
ros2 service call /radar_conti_ars408/set_radar_configuration radar_conti_ars408_msgs/srv/TriggerSetCfg "sensor_id: <sensor_id>"
```

### Configuring Filter Settings

The driver provides a convenient ROS2 service interface to configure various filter parameters. The filter can be configured with the following command

```
ros2 service call /radar_conti_ars408/set_filter radar_conti_ars408_msgs/srv/SetFilter "type: <type>
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

## Configuring Motion Input

This driver optionally lets you input your odometry into the sensor, which will be intepreted by the radars and used in their own tracking algorithms. There are two parameters you need to set:

- `odom_topic_name`: string
- `radar_[index].send_motion`: bool

