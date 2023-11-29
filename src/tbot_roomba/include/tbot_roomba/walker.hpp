#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;


class TBot_Walker : rclcpp::Node {
public:
    TBot_Walker() : Node("tbot_walker") {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Setting up Walker Node...");
        // Create a publisher for the cmd_vel topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // Create a subscription to the laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&TBot_Walker::scan_callback, this, _1));
        RCLCPP_INFO_STREAM(rclcpp::get_logger("tbot_walker"), "Walker Node Initialized!");        
    }
private:
    
};