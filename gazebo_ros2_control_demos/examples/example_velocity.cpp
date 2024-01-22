// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#define _USE_MATH_DEFINES
#include <math.h>

#define NB_OF_SAMPLES 100
#define FREQUENCY 5.0 // Frequency of the sine wave in Hz

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("velocity_test_node");

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10);

  RCLCPP_INFO(node->get_logger(), "node created");

  std_msgs::msg::Float64MultiArray commands;

  using namespace std::chrono_literals;

  // commands.data.push_back(0);
  // for (int i=0; i<100; i++)
  // {
  // publisher->publish(commands);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // for (int i=0; i<100; i++)
  // {
  // commands.data[0] = 1;
  // publisher->publish(commands);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // for (int i=0; i<100; i++)
  // {
  // commands.data[0] = -1;
  // publisher->publish(commands);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // for (int i=0; i<100; i++)
  // {
  // commands.data[0] = 0;
  // publisher->publish(commands);
  // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }


  double angle = 0.0;
  double increment = 2 * M_PI * FREQUENCY / 1000;
  commands.data.push_back(0);
  for (int i = 0; i < 1000; i++)
  {
    commands.data[0] = sin(angle);
    publisher->publish(commands);
    angle += increment;
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Sleep for the duration of one sample
  }
  rclcpp::shutdown();

  return 0;
}
