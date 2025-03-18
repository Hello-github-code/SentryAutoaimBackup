//
// Created by Wang on 23-6-16.
//

#ifndef RMOS_DETECTOR_NODE_HPP
#define RMOS_DETECTOR_NODE_HPP

// ROS
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/bool.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Eigen>
#include <tf2_ros/static_transform_broadcaster.h>

// STD
#include <memory>
#include <string>
#include <vector>

//interfaces
#include "rmos_interfaces/msg/armors.hpp"
#include "rmos_interfaces/msg/armor.hpp"
#include "rmos_interfaces/msg/aimpoint.hpp"
#include "rm2_referee_msgs/msg/robot_status.hpp"
#include "../../Algorithm/include/Dectector/detector_interfaces/detector_interface.hpp"
#include "../../Algorithm/include/Dectector/detector/cj_detector/cj_detector.hpp"
#include "../../Algorithm/include/Dectector/detector/traditional_detector/detector.hpp"
#include "../../Algorithm/include/Dectector/classifier/cj_classifier/cj_classifier.hpp"
#include "../../Algorithm/include/Dectector/classifier/new_classifier/number_classifier.hpp"
#include "../../Algorithm/include/Dectector/solver/pnp_solver/pnp_solver.hpp"
#include "../../Algorithm/include/Debug/debug.hpp"

namespace rmos_detector
{
    class BaseDetectorNode : public rclcpp::Node
    {
    public:
        BaseDetectorNode(const std::string &node_name,
                         const rclcpp::NodeOptions &options) : Node(node_name, options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
        }

        bool is_left_;  // 添加标志位
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        float small_width = 135;
        float small_height = 57;
        float big_width = 225;
        float big_height = 55;

    protected:
        std::shared_ptr<image_transport::Subscriber> image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<rmos_interfaces::msg::Aimpoint>::SharedPtr aim_sub_;
        rclcpp::Subscription<rm2_referee_msgs::msg::RobotStatus>::SharedPtr color_sub_;
        sensor_msgs::msg::CameraInfo camera_info_msg_;
    };

    class BasicDetectorNode : public BaseDetectorNode
    {
    public:
        BasicDetectorNode(const rclcpp::NodeOptions &options);
        
    protected:
        void imageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);

        // std::shared_ptr<detector::CjDetector> cj_detector_;
        std::shared_ptr<detector::Detector> detector_;

        // std::shared_ptr<detector::CjClassifier> cj_classifier_;
        std::shared_ptr<detector::NumberClassifier> number_classfier_;

        std::shared_ptr<detector::PnpSolver> pnp_solver_;

        // 发布器
        rclcpp::Publisher<rmos_interfaces::msg::Armors>::SharedPtr armors_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;

        // 调试信息发布器
        image_transport::CameraPublisher debug_img_pub_;
        image_transport::CameraPublisher debug_bin_img_pub_;
        sensor_msgs::msg::Image::SharedPtr debug_image_msg_;
        sensor_msgs::msg::Image::SharedPtr debug_bin_image_msg_;      
        
        cv::Point2f aim_point_{cv::Point2f(0, 0)};      
    };
} // namespace rmos_detector

#endif //RMOS_DETECTOR_NODE_HPP
