/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief Main function for the walker node
 * @version 0.1
 * @date 2023-11-29
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "tbot_roomba/walker.hpp"
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TBot_Walker>());
    rclcpp::shutdown();
    return 0;
}