//
// Created by Wu on 24-3-14.
//

#include <vector>
#include <Eigen/Dense>
#include "../include/comm_node.hpp"

using namespace std::chrono;

namespace rmos_transporter
{
    UsbCommNode::UsbCommNode(const rclcpp::NodeOptions &options) 
        : CommNode("usb_comm", options)
    {
        this->declare_parameter("is_left", false);
        is_left_ = this->get_parameter("is_left").as_bool();

        std::string node_name = is_left_ ? "usb_comm_l" : "usb_comm_r";
        RCLCPP_INFO(this->get_logger(), "Starting %s node", node_name.c_str());

        std::string quaternion_topic = is_left_ ? "/head_left/quaternion" : "/head_right/quaternion";
        std::string imu_quaternion_topic = is_left_ ? "/imu_quaternion_l" : "/imu_quaternion_r";

        // 回调组和订阅选项
        this->target_sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto target_sub_options = rclcpp::SubscriptionOptions();
        target_sub_options.callback_group = this->target_sub_callback_group_;

        // 订阅者
        this->target_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>
                (quaternion_topic, rclcpp::SensorDataQoS(),
                 std::bind(&UsbCommNode::targetCallBack, this, std::placeholders::_1),
                 target_sub_options);

        // 发布者
        this->quaternion_pub_ = this->create_publisher<rmos_interfaces::msg::QuaternionTime>
                (imu_quaternion_topic, rclcpp::SensorDataQoS());

        // TF 广播器
        this->tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 设置 frame_id
        quaternion_time_msg_.quaternion_stamped.header.frame_id = is_left_ ? "Imu_l" : "Imu_r";
    }

    UsbCommNode::~UsbCommNode()
    {
        // if (recevie_thread_l.joinable())
        //     recevie_thread_l.join();
    }

    void UsbCommNode::targetCallBack(const geometry_msgs::msg::QuaternionStamped::SharedPtr quaternion_time_msg)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->now();
        t.header.frame_id = "Gimbal_Link";
        t.child_frame_id = is_left_ ? "IMU_l" : "IMU_r";
        t.transform.rotation = quaternion_time_msg->quaternion;
        tf_publisher_->sendTransform(t);

        // 发布四元数消息
        quaternion_time_msg_.quaternion_stamped = *quaternion_time_msg;
        quaternion_pub_->publish(quaternion_time_msg_);
    }
} // namespace rmos_transporter

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_transporter::UsbCommNode)
