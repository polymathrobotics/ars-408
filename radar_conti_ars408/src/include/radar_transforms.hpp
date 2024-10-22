#ifndef TRANSFORMS_HPP
#define TRANSFORMS_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <structs.hpp>

namespace radar_transforms
{
  nav_msgs::msg::Odometry transform2DOdom(const nav_msgs::msg::Odometry &odometry, const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, std::string &sensor_frame_id, std::string &base_frame_id, rclcpp::Duration &transform_timeout, rclcpp::Clock::SharedPtr clock)
  {

    geometry_msgs::msg::TransformStamped base_link_to_sensor_transform;
    try
    {
      base_link_to_sensor_transform = tf_buffer->lookupTransform(base_frame_id, sensor_frame_id, odometry.header.stamp, transform_timeout);
    }
    catch (tf2::TimeoutException &exception)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("radar_conti_ars408"), *clock, 1000, "Transform timed out: %s", exception.what());
      return nav_msgs::msg::Odometry{};
    }
    catch (tf2::TransformException &exception)
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("radar_conti_ars408"), *clock, 1000, "Transform failed: %s", exception.what());
      return nav_msgs::msg::Odometry{};
    }
    nav_msgs::msg::Odometry transformed_odometry_base_link;
    nav_msgs::msg::Odometry transformed_odometry_sensor;

    // 1. get velocity of an offset frame, with location defined by transform
    // This is the sensor's velocity oriented in the direction of base_link but the magnitudes are relative to a stationary odom frame
    transformed_odometry_base_link.header = odometry.header;
    transformed_odometry_base_link.header.frame_id = base_link_to_sensor_transform.header.frame_id;
    transformed_odometry_base_link.child_frame_id = base_link_to_sensor_transform.child_frame_id;
    transformed_odometry_base_link.twist.twist.linear.x = odometry.twist.twist.linear.x - odometry.twist.twist.angular.z * base_link_to_sensor_transform.transform.translation.y;
    transformed_odometry_base_link.twist.twist.linear.y = odometry.twist.twist.linear.y + odometry.twist.twist.angular.z * base_link_to_sensor_transform.transform.translation.x;
    transformed_odometry_base_link.twist.twist.angular.z = odometry.twist.twist.angular.z;

    tf2::Quaternion q(
        base_link_to_sensor_transform.transform.rotation.x,
        base_link_to_sensor_transform.transform.rotation.y,
        base_link_to_sensor_transform.transform.rotation.z,
        base_link_to_sensor_transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    transformed_odometry_sensor = transformed_odometry_base_link;
    transformed_odometry_sensor.twist.twist.linear.x = cos(-yaw) * transformed_odometry_base_link.twist.twist.linear.x - sin(-yaw) * transformed_odometry_base_link.twist.twist.linear.y;
    transformed_odometry_sensor.twist.twist.linear.y = sin(-yaw) * transformed_odometry_base_link.twist.twist.linear.x + cos(-yaw) * transformed_odometry_base_link.twist.twist.linear.y;
    transformed_odometry_base_link.twist.twist.angular.z = odometry.twist.twist.angular.z;

    return transformed_odometry_sensor;
  }

  geometry_msgs::msg::Vector3 correctObstacleVelocity(
      nav_msgs::msg::Odometry &corrected_odom,
      const geometry_msgs::msg::Vector3 &raw_obj_velocity,
      const geometry_msgs::msg::Point &raw_obj_pos)
  {
    // Calculate the angular velocity components of object velocity with respect to odom frame
    auto r = hypot(raw_obj_pos.x, raw_obj_pos.y);
    auto tangential_velocity = r * corrected_odom.twist.twist.angular.z;
    auto theta = atan2(raw_obj_pos.y, raw_obj_pos.x);
    auto tangential_velocity_x = tangential_velocity * sin(theta);
    auto tangential_velocity_y = -tangential_velocity * cos(theta);
    geometry_msgs::msg::Vector3 corrected_velocity;
    corrected_velocity.x = raw_obj_velocity.x + corrected_odom.twist.twist.linear.x - tangential_velocity_x;
    corrected_velocity.y = raw_obj_velocity.y + corrected_odom.twist.twist.linear.y - tangential_velocity_y;
    corrected_velocity.z = raw_obj_velocity.z; // Assuming no change in z
    return corrected_velocity;
  }

  radar_conti_ars408_structs::MotionInputSignal createMotionInputSignal(nav_msgs::msg::Odometry &odom, const std::shared_ptr<tf2_ros::Buffer> &tf_buffer, std::string &sensor_frame_id, std::string &base_frame_id, rclcpp::Duration &transform_timeout, rclcpp::Clock::SharedPtr clock)
  {
    auto corrected_odom = radar_transforms::transform2DOdom(odom, tf_buffer, sensor_frame_id, base_frame_id, transform_timeout, clock);

    radar_conti_ars408_structs::MotionInputSignal motion_input_signal;
    // Threshold for standstill detection
    const double standstill_threshold = 0.01;

    motion_input_signal.speed = corrected_odom.twist.twist.linear.x;

    // Calculate direction (0x0 standstill, 0x1 forward, 0x2 backward)
    motion_input_signal.direction = 0x0; // Default to standstill
    if (std::abs(motion_input_signal.speed) > standstill_threshold)
    {
      motion_input_signal.direction = (motion_input_signal.speed > 0.0) ? 0x1 : 0x2; // Forward or backward
    }

    // Calculate yaw rate in deg/s (angular velocity around the z-axis)
    double yaw_rate_rad_per_sec = corrected_odom.twist.twist.angular.z;
    motion_input_signal.yaw_rate = yaw_rate_rad_per_sec * (180.0 / M_PI);

    return motion_input_signal;
  }

}

#endif // TRANSFORMS_HPP