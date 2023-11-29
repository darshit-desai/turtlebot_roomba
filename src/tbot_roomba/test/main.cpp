/**
 * @file main.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Main file for running tests
 * @version 0.1
 * @date 2023-11-29
 * @note This code is derived from the repo:
 * https://github.com/TommyChangUMD/minimal-integration-test/tree/main
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
