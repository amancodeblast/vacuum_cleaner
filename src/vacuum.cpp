// Copyright 2022 Aman Sharma
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/**
 * @file vacuum.cpp
 * @author Aman Kumar SHarma (amankrsharma3@gmail.com)
 * @brief
 * @version 0.1
 * @date 2022-12-07
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @brief Class to implement code for walker behavior
 *
 */
class Walker : public rclcpp::Node {
public:
  /**
   * @brief Constructor
   *
   */
  Walker() : Node("Walker"), collision_distance_(0.5) {
    RCLCPP_INFO(this->get_logger(), "Publisher and Subscriber setup");
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Walker::scan_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Walker starts");
  }

private:
  /**
   * @brief Callback function for moving the turtlebot
   *
   * @param scan_msg
   */
  void
  scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) const {
    int16_t start_idx = 45;
    int16_t end_idx = 315;
    geometry_msgs::msg::Twist cmd_vel_msg;
    double scan_max = scan_msg->range_max;
    double min_dist_to_obstacle = scan_max;

    for (int16_t i = 0; i < int16_t(scan_msg->ranges.size()); i++) {
      if (i <= start_idx || i >= end_idx) {
        if (!std::isnan(scan_msg->ranges[i])) {
          double scan_dist = scan_msg->ranges[i];
          if (scan_dist < min_dist_to_obstacle) {
            min_dist_to_obstacle = scan_dist;
          }
        }
      }
    }
    if (min_dist_to_obstacle <= collision_distance_) {
      RCLCPP_WARN(this->get_logger(), "Obstacle found on path");
      RCLCPP_INFO(this->get_logger(), "Turning the Turtlebot");
      cmd_vel_msg.linear.x = 0.0;
      cmd_vel_msg.angular.z = -0.35;
    } else {
      RCLCPP_INFO(this->get_logger(), "No Obstacles Deteted");
      cmd_vel_msg.linear.x = -0.5;
      cmd_vel_msg.angular.z = 0.0;
    }
    vel_pub_->publish(cmd_vel_msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  double collision_distance_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  std::string cmd_vel_topic = "/cmd_vel";
  std::string laser_topic = "/scan";
};
/**
 * @brief main of the file in order to get the file started
 *
 * @param argc
 * @param argv
 * @return int 0
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walker>());
  rclcpp::shutdown();
  return 0;
}