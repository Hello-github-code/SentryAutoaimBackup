#ifndef RMOS_COMM_NODE_HPP
#define RMOS_COMM_NODE_HPP

#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/quaternion_stamped.hpp>

#include "rmos_interfaces/msg/target.hpp"
#include "rmos_interfaces/msg/quaternion_time.hpp"
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
        CommNode(const std::string &node_name, const rclcpp::NodeOptions &options) : Node(node_name, options) {
            RCLCPP_INFO(this->get_logger(), "Start Communicate Node: %s", node_name.c_str());
        }

    protected:
        bool is_left_;  // 添加标志位
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
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
