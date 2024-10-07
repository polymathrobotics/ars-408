#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

namespace radar_visualization
{
  /**
   * @brief Adds points to the marker to create the radar's FOV shape
   *
   * @param marker Reference to the marker to modify
   * @param min_azimuth Minimum azimuth angle (degrees)
   * @param max_azimuth Maximum azimuth angle (degrees)
   * @param min_distance Minimum range distance (meters)
   * @param max_distance Maximum range distance (meters)
   */
  void addRadarFOVPoints(visualization_msgs::msg::Marker &marker, double min_azimuth, double max_azimuth,
                         double min_distance, double max_distance)
  {
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;

    marker.points.push_back(origin);

    double min_azimuth_rad = min_azimuth * M_PI / 180.0;
    double max_azimuth_rad = max_azimuth * M_PI / 180.0;

    geometry_msgs::msg::Point min_left;
    min_left.x = min_distance * cos(min_azimuth_rad);
    min_left.y = min_distance * sin(min_azimuth_rad);
    min_left.z = 0.0;
    marker.points.push_back(min_left);

    geometry_msgs::msg::Point max_left;
    max_left.x = max_distance * cos(min_azimuth_rad);
    max_left.y = max_distance * sin(min_azimuth_rad);
    max_left.z = 0.0;
    marker.points.push_back(max_left);

    geometry_msgs::msg::Point max_right;
    max_right.x = max_distance * cos(max_azimuth_rad);
    max_right.y = max_distance * sin(max_azimuth_rad);
    max_right.z = 0.0;
    marker.points.push_back(max_right);

    geometry_msgs::msg::Point min_right;
    min_right.x = min_distance * cos(max_azimuth_rad);
    min_right.y = min_distance * sin(max_azimuth_rad);
    min_right.z = 0.0;
    marker.points.push_back(min_right);

    marker.points.push_back(origin);
  }
  /**
   * @brief Creates a line strip marker representing the radar's field of view (FOV)
   *
   * @param radar_frame The reference frame for the radar
   * @param min_azimuth Minimum azimuth angle (degrees)
   * @param max_azimuth Maximum azimuth angle (degrees)
   * @param min_distance Minimum range distance (meters)
   * @param max_distance Maximum range distance (meters)
   * @return visualization_msgs::msg::Marker A line strip marker
   */
  visualization_msgs::msg::Marker createRadarFOVMarker(const std::string &radar_frame,
                                                       double min_azimuth, double max_azimuth,
                                                       double min_distance, double max_distance, rclcpp::Clock::SharedPtr clock)
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = radar_frame;
    marker.header.stamp = clock->now();
    marker.ns = "radar_fov";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set up marker properties
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Add points to represent the radar's FOV
    addRadarFOVPoints(marker, min_azimuth, max_azimuth, min_distance, max_distance);

    return marker;
  }

} // namespace radar_visualization

#endif // VISUALIZATION_HPP
