//
// Created by Wang on 23-6-14.
//

#ifndef RMOS_CAM_NODE_HPP
#define RMOS_CAM_NODE_HPP

//std
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <chrono>

//other
#include <opencv2/core.hpp>

//ros
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.hpp>
#include "rmos_interfaces/msg/target.hpp"
#include "message_filters/subscriber.h"

#include "../../Algorithm/include/Camera/camera_interfaces/camera_interface.hpp"
#include "../../Algorithm/include/Camera/daheng/daheng.hpp"
#include "../../Algorithm/include/Camera/virtual_cam/virtual_cam.hpp"

namespace rmos_cam
{
    class CamNode : public rclcpp::Node
    {
        public:
            CamNode(const std::string & node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
            {
                RCLCPP_INFO(this->get_logger(), "Starting node [%s]", node_name.c_str());
            };

        protected:
            image_transport::CameraPublisher img_pub_;                // 信息发布
            sensor_msgs::msg::CameraInfo camera_info_msg_;            // 相机消息
            sensor_msgs::msg::Image::SharedPtr image_msg_;
            cv::Mat image_;
            cv::Mat image_g;
            std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_manager_;
            rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

            uint32_t frame_id_ = 0;                                   // 帧计数器
    };

    class DahengCamNode : public virtual CamNode
    {
    public:
        DahengCamNode(const rclcpp::NodeOptions & options);
        ~DahengCamNode();
    protected:
        bool is_left_;              // 添加标志位

        std::shared_ptr<camera::DahengCam> cam_dev_;
        std::thread capture_thread_;                    // 采图线程
        GX_OPEN_PARAM open_param_;
        rclcpp::Subscription<rmos_interfaces::msg::Target>::SharedPtr target_sub_;
        rclcpp::CallbackGroup::SharedPtr target_sub_callback_group_;
        int last_target_mode_=0;
        unsigned char lut[256];
        void TargetCallBack(const rmos_interfaces::msg::Target::SharedPtr target);
        void gamma(const cv::Mat & src, cv::Mat &dst)
        {
            src.copyTo(dst);
            cv::MatIterator_<cv::Vec3b> it,end;
            for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
            {
                (*it)[0] = lut[((*it)[0])];
                (*it)[1] = lut[((*it)[1])];
                (*it)[2] = lut[((*it)[2])];
            }
        }
    };

    class VirtualCamNode : public virtual CamNode
    {
    public:
        VirtualCamNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~VirtualCamNode();
    protected:
        std::shared_ptr<camera::VirtualCam> virtual_dev_;
        std::thread capture_thread_;                    // 采图线程
    };
} // namespace rmos_cam

#endif //RMOS_CAM_NODE_HPP
