// Copyright (c) 2023，Horizon Robotics.
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

#ifndef ARM_NODE_H_
#define ARM_NODE_H_

#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>

#include "ai_msgs/msg/perception_targets.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

using rclcpp::NodeOptions;

#define PI 3.1415926f
#define l1 0.1035f
#define l2 0.088f
#define l3 0.17f

class ArmNode : public rclcpp::Node {
 public:

  ArmNode(const std::string &node_name,
        const NodeOptions &options = NodeOptions());

  virtual ~ArmNode();

  rclcpp::TimerBase::SharedPtr timer_;

  // 执行运行流程
  int Run();

  int Encode(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6);

  int Model2D(float x, float y, float alpha);

  int Model(float x, float y, float z);

  int Operate();

  int count_ = 7;

  float targetX_ = 0.15;
  float targetY_ = 0.0;
  float targetZ_ = -0.08;

  std::vector<float> thetas_ = std::vector<float>(6);

  private:
   std::string ai_msg_sub_topic_name_ = "/ai_msg_mono2d_trash_detection";
   rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr
    smart_subscription_ = nullptr;

   std::string serial_data_ = "\x55\x55\x17\x03\x06\xE8\x03\x01\xDC\x05\x02\xDC\x05\x03\xDC\x05\x04\xDC\x05\x05\xDC\x05\x06\xDC\x05";

   std::vector<int> ruler_ = {1074, 1032, 992, 960, 933, 909, 888, 870, 850, 838};

   // 接受Agentnode话题信息
   void SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);
};

#endif  // ARM_NODE_H_