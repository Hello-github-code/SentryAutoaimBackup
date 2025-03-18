#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <cv_bridge/cv_bridge.h>

#include <fstream>
#include <opencv2/opencv.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rmos_recorder {
class RecorderNode : public rclcpp::Node {
public:
  explicit RecorderNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~RecorderNode();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void log_callback(const rcl_interfaces::msg::Log::SharedPtr log_msg);

  // Parameters
  bool is_left_;
  std::string image_raw_topic_;
  std::string rosout_topic_;

  bool record_topic_;
  bool record_log_;
  std::string log_file_path_;

  // ROS-related members
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_subscriber_;
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
};
}  // namespace rmos_recorder

#endif  // RECORDER_HPP
