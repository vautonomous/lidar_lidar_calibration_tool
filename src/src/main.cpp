#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "pcd_extractor_node.h"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PCDExtractorNode>());
    rclcpp::shutdown();
  return 0;
}
