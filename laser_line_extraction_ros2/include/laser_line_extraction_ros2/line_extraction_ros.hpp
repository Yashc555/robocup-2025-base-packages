#ifndef LASER_LINE_EXTRACTION_LINE_EXTRACTION_ROS_HPP_
#define LASER_LINE_EXTRACTION_LINE_EXTRACTION_ROS_HPP_

#include <vector>
#include <string>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "laser_line_extraction_ros2/msg/line_segment.hpp"
#include "laser_line_extraction_ros2/msg/line_segment_list.hpp"
#include "laser_line_extraction_ros2/line_extraction.hpp"

namespace line_extraction
{
class LineExtractionROS : public rclcpp::Node
{
public:
  LineExtractionROS();
  void run();

private:
  // ROS 2 Subscribers and Publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<laser_line_extraction_ros2::msg::LineSegmentList>::SharedPtr line_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  // Parameters
  std::string frame_id_;
  std::string scan_topic_;
  bool pub_markers_;
  bool data_cached_;

  LineExtraction line_extraction_;

  void loadParameters();
  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void cacheData(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  void populateLineSegListMsg(const std::vector<Line> &lines, laser_line_extraction_ros2::msg::LineSegmentList &line_list_msg);
  void populateMarkerMsg(const std::vector<Line> &lines, visualization_msgs::msg::Marker &marker_msg);
};
}
#endif