#ifndef VISUALIZATION_HPP
#define VISUALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath>

namespace radar_visualization
{

  /**
   * A estimated field of view for different object types based on the ARS408's documentation. Some points are duplicated so each array can be sized at 10 elements consistently.
   */
  constexpr std::array<std::pair<int, int>, 10> pedestrian_fov = {{{8, 13}, {22, 20}, {32, 18}, {40, 15}, {50, 7}, {55, 9}, {80, 9}, {110, 0}, {110, 0}, {110, 0}}};

  constexpr std::array<std::pair<int, int>, 10> moped_fov = {{{12, 22}, {23, 26}, {44, 25}, {58, 20}, {63, 12}, {70, 11}, {80, 12}, {120, 13}, {140, 10}, {170, 0}}};

  constexpr std::array<std::pair<int, int>, 10> motorcycle_fov = {{{17, 27}, {42, 35}, {58, 31}, {65, 24}, {70, 11}, {110, 17}, {160, 12}, {190, 13}, {220, 0}, {220, 0}}};

  constexpr std::array<std::pair<int, int>, 10> passenger_car_fov = {{{20, 35}, {53, 45}, {61, 35}, {70, 11}, {149, 24}, {210, 23}, {250, 17}, {250, 0}, {250, 0}, {250, 0}}};

  /**
   * @brief Creates a marker representing the field of view (FOV) for a radar.
   *
   * @param radar_frame The reference frame for the radar
   * @param points The points defining the FOV
   * @param ns The namespace for the marker
   * @param id The ID for the marker
   * @param r Red color component
   * @param g Green color component
   * @param b Blue color component
   * @param a Alpha (transparency) component
   * @param clock Shared pointer to the ROS2 node clock to add the current timestamp to the marker
   * @return visualization_msgs::msg::Marker A marker representing the FOV
   */
  visualization_msgs::msg::Marker
  createFovMarker(const std::string &radar_frame, const std::array<std::pair<int, int>, 10> &points, const std::string &ns, int id, float r, float g, float b, float a, rclcpp::Clock::SharedPtr clock)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = radar_frame;
    marker.header.stamp = clock->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.1; // Line width

    // Set the color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;

    geometry_msgs::msg::Point start_point;
    start_point.x = 0.0;
    start_point.y = 0.0;
    start_point.z = 0.0;

    marker.points.push_back(start_point);
    // Add the original points for the positive y side
    for (const auto &p : points)
    {
      geometry_msgs::msg::Point point;
      point.x = p.first;  // Longitudinal Distance (m)
      point.y = p.second; // Lateral Distance (m)
      point.z = 0.0;      // Flat on the ground
      marker.points.push_back(point);
    }

    // Mirror the points for the negative y side (symmetrical)
    for (auto it = points.rbegin(); it != points.rend(); ++it)
    {
      geometry_msgs::msg::Point point;
      point.x = it->first;     // Longitudinal Distance (m)
      point.y = -(it->second); // Mirror on y-axis (negative lateral distance)
      point.z = 0.0;           // Flat on the ground
      marker.points.push_back(point);
    }

    // Ensure it closes the loop
    marker.points.push_back(start_point);

    return marker;
  }

  visualization_msgs::msg::MarkerArray createRadarFOVMarkers(const std::string &radar_frame,
                                                             rclcpp::Clock::SharedPtr clock)
  {

    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(createFovMarker(radar_frame, pedestrian_fov, "pedestrian", 0, 1.0, 0.0, 1.0, 1.0, clock));       // Purple
    marker_array.markers.push_back(createFovMarker(radar_frame, moped_fov, "moped", 1, 0.0, 1.0, 0.0, 1.0, clock));                 // Green
    marker_array.markers.push_back(createFovMarker(radar_frame, motorcycle_fov, "motorcycle", 2, 0.0, 0.0, 1.0, 1.0, clock));       // Blue
    marker_array.markers.push_back(createFovMarker(radar_frame, passenger_car_fov, "passenger_car", 3, 1.0, 0.0, 0.0, 1.0, clock)); // Red

    return marker_array;
  }

  /**
   * @brief Creates a shaded polygon marker representing the radar's field of view (FOV) using TRIANGLE_LIST.
   *
   * @param radar_frame The reference frame for the radar
   * @param min_azimuth Minimum azimuth angle (degrees)
   * @param max_azimuth Maximum azimuth angle (degrees)
   * @param min_distance Minimum range distance (meters)
   * @param max_distance Maximum range distance (meters)
   * @param clock Shared pointer to the ROS2 node clock to add the current timestamp to the marker
   * @return visualization_msgs::msg::Marker A triangle list marker representing the FOV
   */
  visualization_msgs::msg::Marker createFilteredRadarFOVMarker(const std::string &radar_frame,
                                                               double min_azimuth, double max_azimuth,
                                                               double min_distance, double max_distance,
                                                               rclcpp::Clock::SharedPtr clock)
  {
    visualization_msgs::msg::Marker marker;

    // Header info and frame
    marker.header.frame_id = radar_frame;
    marker.header.stamp = clock->now();
    marker.ns = "radar_fov";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Marker properties
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.25; // Semi-transparent
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Define the origin point (radar position)
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;

    // Convert azimuth angles to radians
    double min_azimuth_rad = min_azimuth * M_PI / 180.0;
    double max_azimuth_rad = max_azimuth * M_PI / 180.0;

    // Define points for the minimum and maximum azimuth and distances
    geometry_msgs::msg::Point min_left, max_left, max_right, min_right;

    min_left.x = min_distance * cos(min_azimuth_rad);
    min_left.y = min_distance * sin(min_azimuth_rad);
    min_left.z = 0.0;

    max_left.x = max_distance * cos(min_azimuth_rad);
    max_left.y = max_distance * sin(min_azimuth_rad);
    max_left.z = 0.0;

    max_right.x = max_distance * cos(max_azimuth_rad);
    max_right.y = max_distance * sin(max_azimuth_rad);
    max_right.z = 0.0;

    min_right.x = min_distance * cos(max_azimuth_rad);
    min_right.y = min_distance * sin(max_azimuth_rad);
    min_right.z = 0.0;

    // Create two triangles to form the radar's FOV:
    // Triangle 1: origin -> min_left -> max_left
    marker.points.push_back(origin);
    marker.points.push_back(min_left);
    marker.points.push_back(max_left);

    // Triangle 2: origin -> max_left -> max_right
    marker.points.push_back(origin);
    marker.points.push_back(max_left);
    marker.points.push_back(max_right);

    // Triangle 3: origin -> max_right -> min_right
    marker.points.push_back(origin);
    marker.points.push_back(max_right);
    marker.points.push_back(min_right);

    // Triangle 4: origin -> min_right -> min_left
    marker.points.push_back(origin);
    marker.points.push_back(min_right);
    marker.points.push_back(min_left);

    return marker;
  }

} // namespace radar_visualization

#endif // VISUALIZATION_HPP
