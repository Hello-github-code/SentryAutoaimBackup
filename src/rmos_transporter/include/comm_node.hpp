//
// Created by Wang on 23-7-9.
//

#ifndef RMOS_COMM_NODE_HPP
#define RMOS_COMM_NODE_HPP

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "rmos_interfaces/msg/target.hpp"
#include "rmos_interfaces/msg/quaternion_time.hpp"
#include "rmos_interfaces/msg/color.hpp"
#include "rmos_interfaces/msg/bullet_speed.hpp"
#include "rmos_interfaces/msg/mode.hpp"
#include "rmos_interfaces/msg/autoaim_state.hpp"
#include "std_msgs/msg/int16.hpp"

#include "Debug/debug.hpp"
#include "Base/const.hpp"
#include "Transporter/usb/usb.hpp"
#include "Transporter/can/can.hpp"

namespace rmos_transporter
{
    class CommNode : public rclcpp::Node
    {
    public:
        CommNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options)
        {
            RCLCPP_INFO(this->get_logger(), "Start Communicate Node: %s", node_name.c_str());
        }

    protected:
        bool is_left_;  // 添加标志位
        rclcpp::Publisher<rmos_interfaces::msg::QuaternionTime>::SharedPtr quaternion_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
        rmos_interfaces::msg::QuaternionTime quaternion_time_msg_;
    };
    
    class UsbCommNode : public CommNode
    {
    public:
        explicit UsbCommNode(const rclcpp::NodeOptions & options);
        ~UsbCommNode();
    protected:
        void targetCallBack(const geometry_msgs::msg::QuaternionStamped::SharedPtr quaternion_time_msg);

        rclcpp::CallbackGroup::SharedPtr target_sub_callback_group_;
        rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr target_sub_;
    };
} // namespace rmos_transporter

#endif //RMOS_COMM_NODE_HPP
