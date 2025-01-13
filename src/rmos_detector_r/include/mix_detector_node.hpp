//
// Created by Wang on 23-6-16.
//

#ifndef RMOS_MIX_DETECTOR_NODE_HPP
#define RMOS_MIX_DETECTOR_NODE_HPP

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
#include "rmos_interfaces/msg/color.hpp"

#include "../../Algorithm/include/Dectector/detector_interfaces/detector_interface.hpp"
#include "../../Algorithm/include/Dectector/detector/cj_detector/cj_detector.hpp"
#include "../../Algorithm/include/Dectector/detector/traditional_detector/detector.hpp"
#include "../../Algorithm/include/Dectector/classifier/cj_classifier/cj_classifier.hpp"
#include "../../Algorithm/include/Dectector/classifier/onnx_classifier/onnx_classifier.hpp"
#include "../../Algorithm/include/Dectector/solver/pnp_solver/pnp_solver.hpp"
#include "../../Algorithm/include/Debug/debug.hpp"
#include "../../Algorithm/include/Dectector/detector/mix_detector/mix_detector.hpp"

namespace rmos_detector_r
{
    class BaseDetectorNode : public rclcpp::Node
    {
    public:
        BaseDetectorNode(const std::string &node_name,
                         const rclcpp::NodeOptions &options) : Node(node_name, options)
        {
            RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
        }

        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;

        std::shared_ptr<image_transport::Subscriber> image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
        rclcpp::Subscription<rmos_interfaces::msg::Color>::SharedPtr color_sub_;

    };

    
    class MixDetectorNode : public BaseDetectorNode
    {
    public:
        MixDetectorNode(const rclcpp::NodeOptions &options) : BaseDetectorNode("mix_detector_r", options)
        {
            RCLCPP_INFO(this->get_logger(), "Starthing mix_detector_r node");
            

            //subscriber
            this->image_sub_ = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(
                    this, "/image_raw_r", std::bind(&MixDetectorNode::imageCallBack, this, std::placeholders::_1),
                    "raw",
                    rmw_qos_profile_sensor_data));
            this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/daheng_camera_info_r", rclcpp::SensorDataQoS(),
                                                                                             [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
                                                                                             {
                                                                                                 RCLCPP_INFO(this->get_logger(), "Receive camera infomation");

                                                                                                 this->camera_info_msg_ = *camera_info_msg;

                                                                                                 this->camera_matrix_.create(3, 3, CV_64FC1);
                                                                                                 this->dist_coeffs_.create(1, 5, CV_64FC1);

                                                                                                 for (int i = 0; i < 9; i++)
                                                                                                 {
                                                                                                     this->camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
                                                                                                 }
                                                                                                 for (int i = 0; i < camera_info_msg->d.size(); i++)
                                                                                                 {
                                                                                                     this->dist_coeffs_.at<double>(0, i) = camera_info_msg->d[i];
                                                                                                 }

                                                                                                 this->camera_info_sub_.reset();

                                                                                             });

            this->color_sub_ = this->create_subscription<rmos_interfaces::msg::Color>
                    ("/color_info_r", rclcpp::SensorDataQoS(), [this](rmos_interfaces::msg::Color::ConstSharedPtr color_msg)
                    {
                        int enemy_color = (*color_msg).color;
                        this->mix_detector_->setEnemyColor(enemy_color);
                    });
            // publisher
            this->armors_pub_ = this->create_publisher<rmos_interfaces::msg::Armors>("/rmos_detector/armors_r", rclcpp::SensorDataQoS());

            //debug info publisher
            debug_img_pub_ = image_transport::create_camera_publisher(this, "/debug_RGB_image_r", rmw_qos_profile_default);
            debug_bin_img_pub_ = image_transport::create_camera_publisher(this, "/debug_bin_image_r", rmw_qos_profile_default);


            //cj_detector_ = std::make_shared<detector::CjDetector>();
            // detector_ = std::make_shared<detector::Detector>();
            //cj_classifier_ = std::make_shared<detector::CjClassifier>();
            pnp_solver_ = std::make_shared<detector::PnpSolver>();
            onnx_classifier_ =  std::make_shared<detector::OnnxClassifier>();
            mix_detector_ = std::make_shared<MixDetect::ArmorDetector>();


            /*publish static TF*/
            this->tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


        }

    protected:
        void imageCallBack(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg);

        //std::shared_ptr<detector::CjDetector> cj_detector_;
        // std::shared_ptr<detector::Detector> detector_;

       // std::shared_ptr<detector::CjClassifier> cj_classifier_;
        std::shared_ptr<detector::OnnxClassifier> onnx_classifier_;

        std::shared_ptr<MixDetect::ArmorDetector> mix_detector_;

        std::shared_ptr<detector::PnpSolver> pnp_solver_;

        /* Publisher */
        rclcpp::Publisher<rmos_interfaces::msg::Armors>::SharedPtr armors_pub_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;

        /*debug*/
        image_transport::CameraPublisher debug_img_pub_;
        image_transport::CameraPublisher debug_bin_img_pub_;
        sensor_msgs::msg::Image::SharedPtr debug_image_msg_;
        sensor_msgs::msg::Image::SharedPtr debug_bin_image_msg_;      
          

        //camera param
        sensor_msgs::msg::CameraInfo camera_info_msg_;

    };


}


#endif //RMOS_DETECTOR_NODE_HPP
