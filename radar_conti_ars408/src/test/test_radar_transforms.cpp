#define CATCH_CONFIG_MAIN
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include <catch2/catch.hpp>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include "radar_transforms.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/quaternion.hpp"

struct TestFixture
{
  TestFixture() {}
  ~TestFixture() {}
};

std::string base_link = "base_link";
std::string radar_link = "radar_link";
std::string odom_link = "odom";

void addTransformToBuffer(
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string &parent_frame,
    const std::string &child_frame,
    const geometry_msgs::msg::Transform &transform,
    const rclcpp::Time &stamp)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.child_frame_id = child_frame;
  transform_stamped.transform = transform;

  tf_buffer->setTransform(transform_stamped, "default_authority", false);
}

std::shared_ptr<tf2_ros::Buffer> initializeBuffer(rclcpp::Clock::SharedPtr clock, const rclcpp::Time &stamp, double radar_yaw = 0.0)
{
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
  tf_buffer->setUsingDedicatedThread(true); // Use a dedicated thread for handling transforms

  // Add transforms directly to the buffer
  geometry_msgs::msg::Transform base_link_to_radar_transform;
  base_link_to_radar_transform.translation.x = 1.0;
  base_link_to_radar_transform.translation.y = 1.0;
  base_link_to_radar_transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, radar_yaw);
  base_link_to_radar_transform.rotation.x = q.x();
  base_link_to_radar_transform.rotation.y = q.y();
  base_link_to_radar_transform.rotation.z = q.z();
  base_link_to_radar_transform.rotation.w = q.w();

  base_link_to_radar_transform.translation.x = 1.0;
  base_link_to_radar_transform.translation.y = 0.0;
  base_link_to_radar_transform.translation.z = 0.0;
  addTransformToBuffer(tf_buffer, base_link, radar_link, base_link_to_radar_transform, stamp);

  geometry_msgs::msg::Transform odom_to_base_link_transform;
  odom_to_base_link_transform.translation.x = 5.0;
  odom_to_base_link_transform.translation.y = 0.0;
  odom_to_base_link_transform.translation.z = 0.0;
  odom_to_base_link_transform.rotation.x = 0.0;
  odom_to_base_link_transform.rotation.y = 0.0;
  odom_to_base_link_transform.rotation.z = 0.0;
  odom_to_base_link_transform.rotation.w = 1.0;

  odom_to_base_link_transform.translation.x = 0.0;
  odom_to_base_link_transform.translation.y = 0.0;
  odom_to_base_link_transform.translation.z = 0.0;
  addTransformToBuffer(tf_buffer, odom_link, base_link, odom_to_base_link_transform, stamp);

  return tf_buffer;
}

TEST_CASE_METHOD(TestFixture, "Obstacle Assertions")
{

  SECTION("Stationary Obstacle and Vehicle")
  {
    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = 0.0;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = 0.0;
    raw_obj_velocity.y = 0.0;
    raw_obj_velocity.z = 0.0;

    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE(corrected_obstacle_velocity.x == raw_obj_velocity.x);
    REQUIRE(corrected_obstacle_velocity.y == raw_obj_velocity.y);
    REQUIRE(corrected_obstacle_velocity.z == raw_obj_velocity.z);
  }
  SECTION("Moving Obstacle and Stationary Vehicle")
  {

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = 0.0;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = 1.0;
    raw_obj_velocity.y = 0.0;
    raw_obj_velocity.z = 0.0;

    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE(corrected_obstacle_velocity.x == raw_obj_velocity.x);
    REQUIRE(corrected_obstacle_velocity.y == raw_obj_velocity.y);
    REQUIRE(corrected_obstacle_velocity.z == raw_obj_velocity.z);
  }

  SECTION("Stationary Obstacle and Linearly Moving Vehicle")
  {

    double linear_velocity = 20.5;

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = linear_velocity;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = -linear_velocity;
    raw_obj_velocity.y = 0.0;
    raw_obj_velocity.z = 0.0;

    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE(corrected_obstacle_velocity.x == 0.0);
    REQUIRE(corrected_obstacle_velocity.y == 0.0);
    REQUIRE(corrected_obstacle_velocity.z == 0.0);
  }

  SECTION("Stationary Obstacle and Angular Moving Vehicle")
  {

    double linear_velocity = 0.0;
    double angular_velocity = 0.25;

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = 0.0;
    vehicle_odometry.twist.twist.angular.z = angular_velocity;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = 0.25;
    raw_obj_velocity.y = -0.50;
    raw_obj_velocity.z = 0.0;

    geometry_msgs::msg::Point raw_obj_pos;
    raw_obj_pos.x = 1.0;
    raw_obj_pos.y = 1.0;
    raw_obj_pos.z = 0.0;

    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE_THAT(corrected_obstacle_velocity.x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(corrected_obstacle_velocity.y, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(corrected_obstacle_velocity.z, Catch::Matchers::WithinAbs(0.0, 1e-12));
  }

  SECTION("Stationary Obstacle and Vehicle with sensor rotated 90 degrees")
  {

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = 0.0;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp, M_PI / 2);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = 0.0;
    raw_obj_velocity.y = 0.0;
    raw_obj_velocity.z = 0.0;
    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE(corrected_obstacle_velocity.x == raw_obj_velocity.x);
    REQUIRE(corrected_obstacle_velocity.y == raw_obj_velocity.y);
    REQUIRE(corrected_obstacle_velocity.z == raw_obj_velocity.z);
  }

  SECTION("Stationary Obstacle and Vehicle moving with sensor rotated pi/3 radians")
  {

    double linear_velocity = 14.0;

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = linear_velocity;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp, M_PI / 3);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    double x_s = cos(-M_PI / 3) * -linear_velocity;
    double y_s = sin(-M_PI / 3) * -linear_velocity;

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = x_s;
    raw_obj_velocity.y = y_s;
    raw_obj_velocity.z = 0.0;
    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE_THAT(corrected_obstacle_velocity.x, Catch::Matchers::WithinAbs(0, 1e-12));
    REQUIRE_THAT(corrected_obstacle_velocity.y, Catch::Matchers::WithinAbs(0, 1e-12));
    REQUIRE(corrected_obstacle_velocity.z == 0.0);
  }

  SECTION("Obstacle and Vehicle moving at the same velocity with sensor rotated pi/3 radians")
  {

    double linear_velocity = 14.0;

    nav_msgs::msg::Odometry vehicle_odometry;
    vehicle_odometry.header.frame_id = odom_link;
    vehicle_odometry.child_frame_id = base_link;
    vehicle_odometry.twist.twist.linear.x = linear_velocity;
    vehicle_odometry.twist.twist.angular.z = 0.0;

    auto clock = rclcpp::Clock::SharedPtr(new rclcpp::Clock());
    rclcpp::Time stamp = clock->now();

    auto tf_buffer = initializeBuffer(clock, stamp, M_PI / 3);

    auto timeout = rclcpp::Duration::from_seconds(0.2);
    auto corrected_odom = radar_transforms::transform2DOdom(vehicle_odometry, tf_buffer, radar_link, base_link, timeout, stamp, clock);

    double x_s = cos(-M_PI / 3) * linear_velocity;
    double y_s = sin(-M_PI / 3) * linear_velocity;

    geometry_msgs::msg::Vector3 raw_obj_velocity;
    raw_obj_velocity.x = 0.0;
    raw_obj_velocity.y = 0.0;
    raw_obj_velocity.z = 0.0;
    geometry_msgs::msg::Point raw_obj_pos;
    auto corrected_obstacle_velocity = radar_transforms::correctObstacleVelocity(corrected_odom, raw_obj_velocity, raw_obj_pos);

    REQUIRE_THAT(corrected_obstacle_velocity.x, Catch::Matchers::WithinAbs(x_s, 1e-12));
    REQUIRE_THAT(corrected_obstacle_velocity.y, Catch::Matchers::WithinAbs(y_s, 1e-12));
    REQUIRE(corrected_obstacle_velocity.z == 0.0);
  }
}
