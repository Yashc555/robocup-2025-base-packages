#include "laser_line_extraction_ros2/line_extraction_ros.hpp"
#include <array>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<line_extraction::LineExtractionROS>();

  double frequency = node->declare_parameter<double>("frequency", 25.0);
  rclcpp::Rate rate(frequency);

  while (rclcpp::ok())
  {
    node->run();
    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}