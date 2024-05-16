# ars-408
Continental ARS 408 Driver for ROS2

# Configuring Filter Settings
The driver provides a convenient ros service interface to configure various filter parameters. The filter can be configured with the following command

```
ros2 service call /set_filter radar_conti_ars408_msgs/srv/SetFilter "type: <type>
index: <index>
min_value: <min_value>
max_value: <max_value>"
```