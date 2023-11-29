/**
 * @file walker.hpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Class definition for the Walker class
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef TBOT_WALKER_HPP_
#define TBOT_WALKER_HPP_
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;


class TBot_Walker : public rclcpp::Node {
public:
    /**
    * @brief Construct a new Walker object
    * 
    */
    TBot_Walker() : Node("Walker_node"), collision_threshold_(0.4), random_turn(-1) {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Setting up Walker Node...");
        // Set the collision threshold
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Collision Threshold: " << collision_threshold_);
        // Create a publisher for the cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // Create a subscription to the laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&TBot_Walker::scan_callback, this, _1));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Walker Node Initialized!");        
    }
private:
    /**
     * @brief Function to handle the laser scan data
     * 
     * @param msg 
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    /**
     * @brief Function to publish the velocity commands
     * 
     * @param msg 
     */
    void publish_velocity(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    double collision_threshold_;
    int random_turn;

};
#endif  // TBOT_WALKER_HPP_