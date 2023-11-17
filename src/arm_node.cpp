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

#include "include/arm_node.h"

ArmNode::ArmNode(const std::string &node_name, const NodeOptions &options)
    : rclcpp::Node(node_name, options) {

  this->declare_parameter<float>("targetX", targetX_);
  this->declare_parameter<float>("targetY", targetY_);
  this->declare_parameter<float>("targetZ", targetZ_);

  this->get_parameter<float>("targetX", targetX_);
  this->get_parameter<float>("targetY", targetY_);
  this->get_parameter<float>("targetZ", targetZ_);

  {
    std::stringstream ss;
    ss << "Parameter:"
       << "\n targetX: " << targetX_
       << "\n targetY: " << targetY_
       << "\n targetZ: " << targetZ_;
    RCLCPP_WARN(rclcpp::get_logger("arm_node"), "%s", ss.str().c_str());
  }

  RCLCPP_WARN(rclcpp::get_logger("arm_node"), "ArmNode Init Succeed!");

  timer_ = create_wall_timer(std::chrono::seconds(3),
                              std::bind(&ArmNode::Run, this));

  smart_subscription_ =
    this->create_subscription<ai_msgs::msg::PerceptionTargets>(
        ai_msg_sub_topic_name_,
        10,
        std::bind(&ArmNode::SmartTopicCallback,
                  this,
                  std::placeholders::_1));
}

ArmNode::~ArmNode() {}

int ArmNode::Run() {
  
  switch(count_){
    case 0: Encode(0, 90, -90, 0, 0, 0, 0); count_ += 1; break;
    case 1: Model(targetX_, targetY_, targetZ_); Encode(thetas_[0], thetas_[1], thetas_[2], thetas_[3], 0, 0, 0); count_ += 1; break;
    case 2: Encode(thetas_[0], thetas_[1], thetas_[2], thetas_[3], 0, 0, 1); count_ += 1; break;
    case 3: Encode(0, 90, -90, 0, 0, 0, 1); count_ += 1; break;
    case 4: Encode(-90, thetas_[1], thetas_[2], thetas_[3], 0, 0, 1); count_ += 1; break;
    case 5: Encode(-90, thetas_[1], thetas_[2], thetas_[3], 0, 0, 0); count_ += 1; break;
    case 6: Encode(0, 90, -90, 0, 0, 0, 0); count_ += 1; break;
  }

  Operate();
  
  return 0;
}

int ArmNode::Operate() {

  std::ofstream serial("/dev/ttyUSB0");  // 替换为您的USB串口设备路径

  if (!serial.is_open()) {
      std::cerr << "无法打开串口" << std::endl;
      RCLCPP_ERROR(rclcpp::get_logger("arm_node"), "ERROR!");
  }

  serial << serial_data_;  // 将数据写入串口

  serial.close();  // 关闭串口

  RCLCPP_WARN(rclcpp::get_logger("arm_node"), "Operate Succeed!");
  return 0;
}

std::string decimalToHex(int decimalNumber) {
  unsigned char lowByte = decimalNumber & 0xFF; // 获取低八位
  unsigned char highByte = (decimalNumber >> 8) & 0xFF; // 获取高八位

  std::string data = "";
  data.push_back(lowByte);
  data.push_back(highByte);

  return data;
}

int ArmNode::Encode(float theta0, float theta1, float theta2, float theta3, float theta4, float theta5, float theta6){

  int num0 = 2000 * (90.0 + theta0) / 180.0 + 500.0;
  int num1 = 2000 * (180.0 - theta1) / 180.0 + 500.0;
  int num2 = 2000 * (90.0 + theta2) / 180.0 + 500.0;
  int num3 = 2000 * (90.0 + theta3) / 180.0 + 500.0;

  int num6 = 1500;
  if(theta6 == 0){
    num6 = 500;
  } else if (theta6 == 1){
    num6 = 1500;
  }

  std::stringstream ss;
  ss << "Each angle:"
      << "\n theta1: " << theta1
      << " theta1 num: " << num1
      << "\n theta2: " << theta2
      << " theta2 num: " << num2
      << "\n theta3: " << theta3
      << " theta3 num: " << num3;

  RCLCPP_INFO(rclcpp::get_logger("arm_node"), "%s", ss.str().c_str());

  unsigned char cmd = 3;
  unsigned char motor_length = 5;
  unsigned char data_length = 5 + motor_length * 3;
  int time = 2000;

  std::string data = "\x55\x55";
  data.push_back(data_length);
  data.push_back(cmd);
  data.push_back(motor_length);

  data = data + decimalToHex(time);
  data = data + "\x03" + decimalToHex(num3);
  data = data + "\x04" + decimalToHex(num2);
  data = data + "\x05" + decimalToHex(num1);
  data = data + "\x06" + decimalToHex(num0);

  data = data + "\x01" + decimalToHex(num6);

  serial_data_ = data;

  return 0;
}

int ArmNode::Model2D(float x, float y, float alpha) {
  float m,n,k,a,b,c;
  float theta1,theta2,theta3,s1ps2;

  m = l3 * cos(alpha) - x;
  n = l3 * sin(alpha) - y;

  k = (l2 * l2 - l1 * l1 - m * m - n * n) / (2 * l1);//中间变量

  a = m * m + n * n;             //解一元二次方程
  b = - 2 * n * k;
  c = k * k - m * m;

  if(b * b - 4 * a * c <= 0)   //b^2-4ac 小于0即无实根，直接返回
  { 
    return 1; //返回1， 作错误
  }
  
  theta1 = (-b + sqrt(b * b - 4 * a * c )) / 2 / a;  //求解二元一次方程，只取其中一个，另外一个解是(-b+sqrt(b*b-4*a*c))/2/a
  theta1 = asin(theta1) * 180 / PI;       //将弧度换算为角度  

  //限制最大角度为正负90度
  if (theta1 > 90 || theta1 < -90) {
    return 2;
  }

  k = (l1 * l1 - l2 * l2 - m * m - n * n)/2/l2;     
  a = m * m + n * n;                        //解一元二次方程
  b = - 2 * n * k;
  c = k * k - m * m;
  
  if(b * b - 4 * a * c <= 0)   //方程无实根就不做求解
  {
    return 3;          //返回2， 作错误标记
  }

  s1ps2 = (-b - sqrt(b * b - 4 * a * c)) / 2 / a;      
  s1ps2 = asin(s1ps2) * 180 / PI;          //将弧度换算为角度  

  if (s1ps2 > 90 || s1ps2 < -90) {
    return 4;
  }

  theta2=s1ps2-theta1;      
  //限制最大角度为正负90度
  if (theta2 > 90 || theta2 < -90) {
    return 5;
  }

  theta3=alpha*180/PI-theta1-theta2;   //求5号舵机角度
  //控制舵机的最大角度180
  if (theta3 > 90 || theta3 < -90) {
    return 5;
  }
  
  // 将求得的三个角度，转换为对应的脉宽，然后控制舵机转动。
  // 需要注意的是机械臂的舵机开机后为1500位置。我们将此位置定为0度，即0度为180度舵机的90度位置。
  // 因舵机安装方向的不同，舵机角度的正负方向要根据舵机安装方向调整
  // 将舵机角度转为公式为
  //   (2000 * 角度 / 180 + 500)
  // 上式的角度的范围是0-180度

  float xo = l1*cos(theta1 * PI / 180) + l2*cos(theta1 * PI / 180 + theta2 * PI / 180) + l3*cos(theta1 * PI / 180 + theta2 * PI / 180 + theta3 * PI / 180);
  float yo = l1*sin(theta1 * PI / 180) + l2*sin(theta1 * PI / 180 + theta2 * PI / 180) + l3*sin(theta1 * PI / 180 + theta2 * PI / 180 + theta3 * PI / 180);
  
  if (abs(xo - x) > 0.001 || abs(yo - y) > 0.001) {
    return 6;
  }

  thetas_[1] = theta1;
  thetas_[2] = theta2;
  thetas_[3] = theta3;

  return 0; //一切正常返回0

}

int ArmNode::Model(float x, float y, float z) {

  float alpha = 0;
  int ret = -1;
  
  while(ret != 0 and alpha > -135){
    ret = Model2D(x, z, alpha / 180 * PI);
    alpha -= 1;
  }
  thetas_[0] = atan(y / x);

  return ret;
}


void ArmNode::SmartTopicCallback(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {

  std::stringstream ss;
  ss << "Recved ai msg"
     << ", frame_id: " << msg->header.frame_id
     << ", stamp: " << msg->header.stamp.sec << "_" << msg->header.stamp.nanosec
     << ", targets size: " << msg->targets.size() << "\n";

  for (const auto& tar : msg->targets) {
    ss << " has roi num: " << tar.rois.size();
    
    for (const auto& roi : tar.rois) {

      if (roi.rect.x_offset >= 720 && (roi.rect.x_offset + roi.rect.width) <= 1200
        && roi.rect.y_offset >= 540) {
          ss << ", roi type: " << roi.type;
          ss << ", roi x1: " << roi.rect.x_offset;
          ss << ", roi y1: " << roi.rect.y_offset;
          ss << ", roi x2: " << roi.rect.x_offset + roi.rect.width;
          ss << ", roi y2: " << roi.rect.y_offset + roi.rect.height;

          
          for (int i = 0; i < ruler_.size(); i++) {
            if (roi.rect.y_offset + roi.rect.height > ruler_[i] && count_ == 7) {

              int tmp = i + 16;
              targetX_ = static_cast<float>(tmp) / 100;
              ss << ", target trash x = " << targetX_ << " m.";
              count_ = 0;
              break;
            }
          }

          
        }
    }

    ss << "\n";
  }

  RCLCPP_INFO(rclcpp::get_logger("TrashGradNode"), "%s", ss.str().c_str());

}