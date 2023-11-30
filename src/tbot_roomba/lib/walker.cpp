/**
 * @file walker.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Library implementation for the Walker class
 * @version 0.1
 * @date 2023-11-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <random>
#include <tbot_roomba/walker.hpp>
/**
 * @brief Function to handle the publish velocity commands
 *
 * @param msg
 */
void TBot_Walker::publish_velocity(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  publisher_->publish(*msg);
}
/**
 * @brief Function to handle the laser scan data
 *
 * @param msg
 */
void TBot_Walker::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  int16_t min_angle = 45.0;
  int16_t max_angle = 315.0;
  double max_distance = msg->range_max;
  double min_distance_to_obstacle = max_distance;
  for (int16_t i = 0; i < int16_t(msg->ranges.size()); i++) {
    if (i <= min_angle || i >= max_angle) {
      if (!std::isnan(msg->ranges[i])) {
        double scan_dist = msg->ranges[i];
        if (scan_dist < min_distance_to_obstacle) {
          min_distance_to_obstacle = scan_dist;
        }
      }
    }
  }
  geometry_msgs::msg::Twist cmd_vel_msg;
  if (min_distance_to_obstacle <= collision_threshold_) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("tbot_walker"),
                       "Collision Imminent!");

    cmd_vel_msg.linear.x = 0.0;
    if (random_turn == -1) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Turning Right!");
    } else {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Turning Left!");
    }
    cmd_vel_msg.angular.z = random_turn * 1.0;
  } else {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1);
    random_turn = (dis(gen) == 0) ? -1 : 1;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Moving Forward!");
    cmd_vel_msg.linear.x = 0.5;
    cmd_vel_msg.angular.z = 0.0;
  }
  // Call the publish_velocity function
  publish_velocity(std::make_shared<geometry_msgs::msg::Twist>(cmd_vel_msg));
}
