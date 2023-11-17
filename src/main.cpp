#include <cstdio>

#include "include/arm_node.h"

int main(int argc, char** argv) {
  std::stringstream ss;
  ss << "\n\tThis is hobot arm package.\n\n"
     << "============================================\n"
     << "\tthe robotic arm device\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<ArmNode>("arm_node");
  rclcpp::spin(node);

  return 0;
}