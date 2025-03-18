//
// Created by Wang on 23-6-14.
//

//std
#include <chrono>
#include <sstream>

//ros
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>

#include "../include/cam_node.hpp"

namespace rmos_cam
{
    DahengCamNode::DahengCamNode(const rclcpp::NodeOptions &options) : CamNode("daheng_camera", options)
    {
        // 从参数获取 is_left
        this->declare_parameter("is_left", false);
        is_left_ = this->get_parameter("is_left").as_bool();

        std::string node_name = is_left_ ? "daheng_cam_l" : "daheng_cam_r";
        RCLCPP_INFO(this->get_logger(), "Starting %s node", node_name.c_str());

        cam_dev_ = std::make_shared<camera::DahengCam>();

        // 根据 is_left_ 选择配置文件
        std::string config_file = is_left_ ? 
            "./rmos_bringup/configure/camera_left/daheng_camera_l.xml" :
            "./rmos_bringup/configure/camera_right/daheng_camera_r.xml";
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Open daheng_camera.xml fail!");
            exit(0);
        }

        int Width, Height, Exposure, RGain, GGain, BGain, Gamma, Fps;
        int AutoExposure, AutoWhiteBalance;

        fs["width"] >> Width;
        fs["height"] >> Height;
        fs["exposure"] >> Exposure;
        fs["rgain"] >> RGain;
        fs["ggain"] >> GGain;
        fs["bgain"] >> BGain;
        fs["fps"] >> Fps;
        fs["gain"] >> Gamma;

        // 根据 is_left_ 选择相机序列号
        char* sn = is_left_ ? (char*)"FDF22130004" : (char*)"NF0190070003";
        open_param_.pszContent = sn;
        open_param_.openMode = 0;
        open_param_.accessMode = 3;

        // set paramter
        cam_dev_->set_parameter(camera::CamParamType::Height, Height);
        cam_dev_->set_parameter(camera::CamParamType::Width, Width);
        cam_dev_->set_parameter(camera::CamParamType::AutoExposure, AutoExposure);
        cam_dev_->set_parameter(camera::CamParamType::Exposure, Exposure);
        cam_dev_->set_parameter(camera::CamParamType::AutoWhiteBalance, AutoWhiteBalance);
        cam_dev_->set_parameter(camera::CamParamType::RGain, RGain);
        cam_dev_->set_parameter(camera::CamParamType::GGain, GGain);
        cam_dev_->set_parameter(camera::CamParamType::BGain, BGain);
        cam_dev_->set_parameter(camera::CamParamType::Gamma, Gamma);
        cam_dev_->set_parameter(camera::CamParamType::Fps, Fps);

        cam_dev_->open(&open_param_);
        for (int i = 0; i < 256; i++) {
            float normalize = (float)(i/255.0);
            lut[i] = cv::saturate_cast<uchar>(pow(normalize,0.6) * 255.0f);
        }

        // 根据 is_left_ 选择话题名和frame_id
        std::string img_topic = is_left_ ? "/image_raw_l" : "/image_raw_r";
        std::string frame_id = is_left_ ? "LCam_Link" : "RCam_Link";
        std::string info_topic = is_left_ ? "/daheng_camera_info_l" : "/daheng_camera_info_r";
        
        img_pub_ = image_transport::create_camera_publisher(this, img_topic, rmw_qos_profile_default);

        // 加载相机信息
        cam_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "DahengCam");
        auto pkg_path = ament_index_cpp::get_package_share_directory("rmos_bringup");
        auto yaml_path = is_left_ ?
            "file://" + pkg_path + "/configure/camera_left/daheng_cam_info_l.yaml" :
            "file://" + pkg_path + "/configure/camera_right/daheng_cam_info_r.yaml";

        if (!cam_info_manager_->loadCameraInfo(yaml_path)) {
            RCLCPP_WARN(this->get_logger(), "Load Camera Info Fail!");
        } else {
            camera_info_msg_ = cam_info_manager_->getCameraInfo();
        }

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, 10);

        capture_thread_ = std::thread{[this, frame_id]() -> void
        {
            while (rclcpp::ok())
            {
                if (!cam_dev_->is_open()) {
                    exit(0);
                }

                if (cam_dev_->grab_image(image_)) {
                    gamma(image_, image_g);
                    image_msg_ = cv_bridge::CvImage(std_msgs::msg::Header(),"bgr8",image_g).toImageMsg();
                    (*image_msg_).header.stamp = camera_info_msg_.header.stamp = this->now() - rclcpp::Duration(0, 23 * 1e6);
                    (*image_msg_).header.frame_id = frame_id;
                    camera_info_msg_.header.frame_id = frame_id;

                    img_pub_.publish(*image_msg_, camera_info_msg_);
                    camera_info_pub_->publish(camera_info_msg_);
                } else {
                    std::cout << cam_dev_->error_message() << std::endl;
                    cam_dev_->close();
                    cam_dev_->open(&open_param_);
                }
            }
        }};
    }

    DahengCamNode::~DahengCamNode()
    {
        if (capture_thread_.joinable())
        {
            capture_thread_.join();
        }
        cam_dev_->close();
        RCLCPP_INFO(this->get_logger(), "Camera node destroyed!");
    }
} // namespace rmos_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rmos_cam::DahengCamNode)
