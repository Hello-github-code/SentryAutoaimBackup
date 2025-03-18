#include "../include/recorder_node.hpp"

namespace rmos_recorder {
RecorderNode::RecorderNode(const rclcpp::NodeOptions &options) : Node("recorder_node", options) {
  this->declare_parameter("is_left", false);
  this->declare_parameter("record_topic", true);
  this->declare_parameter("record_log", true);
  is_left_ = this->get_parameter("is_left").as_bool();
  record_topic_ = this->get_parameter("record_topic").as_bool();
  record_log_ = this->get_parameter("record_log").as_bool();

  std::string node_name = is_left_ ? "recorder_l" : "recorder_r";
  RCLCPP_INFO(this->get_logger(), "Starting %s node", node_name.c_str());

  image_raw_topic_ = is_left_ ? "/image_raw_l" : "/image_raw_r";
  rosout_topic_ = is_left_ ? "/rosout_l" : "/rosout_r";

  // Initialize rosbag2 writer
  writer_ = std::make_shared<rosbag2_cpp::Writer>();

  // 获取当前日期时间并设置为 URI
  std::ostringstream oss;
  std::time_t t = std::time(nullptr);
  std::tm tm;
#ifdef _WIN32
  localtime_s(&tm, &t);  // Windows平台
#else
  localtime_r(&t, &tm);  // Linux/Unix平台
#endif
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  std::string current_time_str = oss.str();

  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = "rosbag2/rosbag2_" + current_time_str;
  storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer_->open(storage_options, converter_options);

  // Subscribe to /image_raw
  if (record_topic_) {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_raw_topic_, 10, std::bind(&RecorderNode::image_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Recording topic: /image_raw");
  } else {
    RCLCPP_WARN(this->get_logger(), "Topic recording is disabled.");
  }

  // Subscribe to /rosout
  if (record_log_) {
    log_subscriber_ = this->create_subscription<rcl_interfaces::msg::Log>(
      rosout_topic_, 10, std::bind(&RecorderNode::log_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Recording logs from /rosout");
  } else {
    RCLCPP_WARN(this->get_logger(), "Log recording is disabled.");
  }
}

RecorderNode::~RecorderNode() { RCLCPP_INFO(this->get_logger(), "RecorderNode shutting down."); }

void RecorderNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  if (record_topic_) {
    try {
      // Convert ROS image message to OpenCV format
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      cv::Mat compressed_image;

      // Compress image to JPEG
      std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};  // Quality 90%
      std::vector<uchar> buffer;
      cv::imencode(".jpg", cv_ptr->image, buffer, compression_params);

      // Create compressed image message
      auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      compressed_msg->header = msg->header;
      compressed_msg->format = "jpeg";
      compressed_msg->data = buffer;

      std::string compressed_topic = is_left_ ? "/image_raw_l/compressed" : "/image_raw_r/compressed";
      writer_->write(*compressed_msg, compressed_topic, compressed_msg->header.stamp);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to process image: %s", e.what());
    }
  }
}

void RecorderNode::log_callback(const rcl_interfaces::msg::Log::SharedPtr log_msg) {
  if (record_log_) {
    // Write log message to rosbag
    writer_->write(*log_msg, rosout_topic_, log_msg->stamp);
  }
}
}  // namespace rmos_recorder

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_recorder::RecorderNode)
