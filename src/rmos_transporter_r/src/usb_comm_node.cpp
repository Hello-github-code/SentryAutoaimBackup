//
// Created by Wu on 24-3-14.
//

#include <vector>
#include <Eigen/Dense>
#include "../include/comm_node.hpp"

using namespace std::chrono;

namespace rmos_transporter_r
{
    UsbCommNode::UsbCommNode(const rclcpp::NodeOptions &options) : CommNode("usb_comm_r", options)
    {

        std::map<std::string, int> transporter_params {
            {"interface_usb_vid", 0},
            {"interface_usb_pid", 0},
            {"interface_usb_read_endpoint", 0},
            {"interface_usb_write_endpoint", 0},
            {"interface_usb_read_timeout", 0},
            {"interface_usb_write_timeout", 0},
        };

        this->declare_parameters("", transporter_params);
        // transporter parameter
        this->get_parameter<int>("interface_usb_vid", interface_usb_vid_);
        this->get_parameter<int>("interface_usb_pid", interface_usb_pid_);
        this->get_parameter<int>("interface_usb_read_endpoint", interface_usb_read_endpoint_);
        this->get_parameter<int>("interface_usb_write_endpoint", interface_usb_write_endpoint_);
        this->get_parameter<int>("interface_usb_read_timeout", interface_usb_read_timeout_);
        this->get_parameter<int>("interface_usb_write_timeout", interface_usb_write_timeout_);

        // create callback group
        this->receive_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        this->target_sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        // create publisher and subscriber
        this->tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        auto target_sub_options = rclcpp::SubscriptionOptions();
        target_sub_options.callback_group = this->target_sub_callback_group_;
        this->target_sub_ = this->create_subscription<rmos_interfaces::msg::Target>
                ("/target_r", rclcpp::SensorDataQoS(),
                 std::bind(&UsbCommNode::targetCallBack, this, std::placeholders::_1),
                 target_sub_options);
        this->target_sub_l_ = this->create_subscription<rmos_interfaces::msg::Target>
                ("/target_l", rclcpp::SensorDataQoS(),
                 std::bind(&UsbCommNode::othertargetstateCallBack, this, std::placeholders::_1),
                 target_sub_options);
        this->perception_target_sub_ = this->create_subscription<std_msgs::msg::Int16>
            ("/perception_target_l", rclcpp::SensorDataQoS(),
            std::bind(&UsbCommNode::perceptionTargetCallBack, this, std::placeholders::_1),
                target_sub_options);

                 
        this->quaternion_pub_ = this->create_publisher<rmos_interfaces::msg::QuaternionTime>("/imu_quaternion_r", rclcpp::SensorDataQoS());


        this->receive_timer_ = this->create_wall_timer(1ms, std::bind(&UsbCommNode::recevieCallBack, this));
        // 可尝试定时器会不会出问题 再用线程
        // this->recevie_thread_r = std::thread(&UsbCommNode::recevieCallBack, this);

        quaternion_time_msg_.quaternion_stamped.header.frame_id = std::string("Imu_r");

        interface_usb_pid_=0x573f;
        interface_usb_vid_=0x0483;
        interface_usb_read_endpoint_= 0x81;
        interface_usb_write_endpoint_= 0x01;
        interface_usb_read_timeout_=1;
        interface_usb_write_timeout_= 1;

        RCLCPP_INFO(this->get_logger(), "Init Transporter");
        transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
            interface_usb_vid_, 
            interface_usb_pid_, 
            interface_usb_read_endpoint_, 
            interface_usb_write_endpoint_, 
            interface_usb_read_timeout_, 
            interface_usb_write_timeout_
        );

        RCLCPP_INFO(this->get_logger(), "Open Transporter");
        if (transporter_->open() == true) {
            RCLCPP_INFO(this->get_logger(), "Success");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "FAILED!!!");
        }
        RCLCPP_INFO(this->get_logger(), "Finish Init");

    }

    UsbCommNode::~UsbCommNode()
    {
        // if (recevie_thread_r.joinable())
        //     recevie_thread_r.join();
    }

    void UsbCommNode::targetCallBack(const rmos_interfaces::msg::Target::SharedPtr target)
    {
        if(debug::get_debug_option(base::SHOW_TOTAL_COST))
        {
            if(target->gun_pitch!=0)
            {
                rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
                auto time = steady_clock_.now();
                RCLCPP_INFO(this->get_logger(), "Cost %.4f ms",(time.seconds()-last_time_) * 1000);
                last_time_ = time.seconds();
            }
        }
        send_package_._SOF = 0x55;
        send_package_._EOF = 0xFF;
        send_package_.ID = RMOS_0_SEND_ID;
        if(int(target->id) == 255 && (int16_t)(target->gun_pitch) == 0 && perception_yaw_ != 0 && perception_yaw_ < 180 && perception_yaw_ > -180)
        {
            this->perception_target2state(&(send_package_.AimbotState));
            send_package_.PitchRelativeAngle = (int16_t)(0);
            send_package_.YawRelativeAngle = (int16_t)(perception_yaw_ * 32768.0/180);
            //std::cout << "r_perception_send:" << (int16_t)(perception_yaw_);
            send_package_.SystemTimer = (int16_t)(target->timestamp_recv);
            transporter_->write((unsigned char *)&send_package_, sizeof(transporter::RMOSSendPackage));
        }
        else{
            this->target2state(target, &(send_package_.AimbotState));
            send_package_.PitchRelativeAngle = (float)(target->gun_pitch );
            send_package_.YawRelativeAngle = (float)(target->gun_yaw );
            send_package_.SystemTimer = (int16_t)(target->timestamp_recv);
            transporter_->write((unsigned char *)&send_package_, sizeof(transporter::RMOSSendPackage));
        }
        // this->target2state(target, &(send_package_.AimbotState));
        // send_package_.PitchRelativeAngle = (int16_t)(target->gun_pitch * 32768.0 / 180.0);
        // send_package_.YawRelativeAngle = (int16_t)(target->gun_yaw * 32768.0 / 180.0);
        // send_package_.SystemTimer = (uint32_t)(target->timestamp_recv);
        // transporter_->write((unsigned char *)&send_package_, sizeof(transporter::RMOSSendPackage));
    }
     void UsbCommNode::othertargetstateCallBack(const rmos_interfaces::msg::Target::SharedPtr target)
    {
        if(target->track_state==0)
        another_target_state=false;
        else
        another_target_state=true;
    }

    void UsbCommNode::target2state(const rmos_interfaces::msg::Target::SharedPtr target, u_char *buf)
    {
        memset(buf, 0, 3);
        switch ((base::TrackState)target->track_state)
        {
            case base::TrackState::TRACKING:
            case base::TrackState::TEMP_LOST:
                buf[0] |= 0x01;
                break;
            default:
                break;//00000001
                      //00100000
                      //00100001
        }
        if(target->mode==1)
        buf[0] |= 0x08;
        else if(target->mode==2)
        buf[0] |= 0x10;
        if(another_target_state==true)
                buf[0] |= 0x04;
        switch(target->outpost_direction)
        {
            case 1:
                buf[0] |= 0x40;
            case -1:
                buf[0] |= 0;

        }
        if (target->suggest_fire)
            buf[0] |= 0x02;
        switch (target->id)
        {
            case 0:
                buf[1] |= 0x80;//01000000
                break;
            case 1:
                buf[1] |= 0x01;//00000001
                break;
            case 2:
                buf[1] |= 0x02;//00000010
                break;
            case 3:
            case 11:
                buf[1] |= 0x04;//00000100
                break;
            case 4:
            case 12:
                buf[1] |= 0x08;//00001000
                break;
            case 5:
                buf[1] |= 0x10;//00010000
                break;
            case 6:
                buf[1] |= 0x40;
                break;
            case 7:
                buf[1] |= 0x20;
                break;
            default:
                break;
        }

        double distance = sqrt(target->position.x *  target->position.x +
                               target->position.y *  target->position.y +
                               target->position.z *  target->position.z);

        if(distance<3){

            buf[2] |= 0x01;

        }
        else if(3< distance&&distance<5){

            buf[2] |= 0x02;

        }
        else{

            buf[2] |= 0x03;

        }
    }

    void UsbCommNode::recevieCallBack()
    {
        //while (rclcpp::ok())
     // {
                uint8_t receive_package[64];

        int read_size = transporter_->read(receive_package, 64);
         //RCLCPP_INFO(this->get_logger(), "read size : %d", read_size);
           //std::cout<<"recevie///////////////////////////////////////"<<'\n';
        switch (receive_package[1])
        {
            
            case RMOS_REFEREE_RECEIVE_ID:
            {
                transporter::RMOSRefereeReceivePackage package;
                memcpy(&package, receive_package, 
                    sizeof(transporter::RMOSRefereeReceivePackage));

                // color
                if (package.robot_id < 10) {
                    color_msg_.color = (int) (base::Color::BLUE);
                } else if (package.robot_id > 100) {
                    color_msg_.color = (int) base::Color::RED;
                } else {
                    color_msg_.color = (int) (base::Color::BLUE);
                }
                this->color_pub_->publish(color_msg_);
                //bs
                bs_msg_l_.speed=package.bs_l;
                bs_msg_r_.speed=package.bs_r;
                this->bs_pub_l_->publish(bs_msg_l_);
                this->bs_pub_r_->publish(bs_msg_r_);
                // TODO : 除哨兵外的车需要的信息

                break;
            }
            case RMOS_IMU_0_RECEIVE_ID:
            {
                transporter::RMOSIMUReceivePackage package;
                memcpy(&package, receive_package, 
                    sizeof(transporter::RMOSIMUReceivePackage));
                int time_offset = 25000;
                quaternion_time_msg_.quaternion_stamped.header.stamp = this->now() + rclcpp::Duration(0,time_offset);
                quaternion_time_msg_.quaternion_stamped.quaternion.w = (double)package.q0;
                quaternion_time_msg_.quaternion_stamped.quaternion.x = (double)package.q1;
                quaternion_time_msg_.quaternion_stamped.quaternion.y = (double)package.q2;
                quaternion_time_msg_.quaternion_stamped.quaternion.z = (double)package.q3;
                // std::cout<<"q0"<<package.q0<<'\n';
                // std::cout<<"q1"<<package.q1<<'\n';
                // std::cout<<"q2"<<package.q2<<'\n';
                // std::cout<<"q3"<<package.q3<<'\n';
                        
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp =  quaternion_time_msg_.quaternion_stamped.header.stamp;
                t.header.frame_id = "Gimbal_Link"; //注意坐标系
                t.child_frame_id = "IMU_r";
                t.transform.rotation.x = quaternion_time_msg_.quaternion_stamped.quaternion.x;
                t.transform.rotation.y = quaternion_time_msg_.quaternion_stamped.quaternion.y;
                t.transform.rotation.z = quaternion_time_msg_.quaternion_stamped.quaternion.z;
                t.transform.rotation.w = quaternion_time_msg_.quaternion_stamped.quaternion.w;
                tf_publisher_->sendTransform(t);
                memcpy(&quaternion_time_msg_.timestamp_recv, &package.TimeStamp, 4);
                    this->quaternion_pub_->publish(quaternion_time_msg_);
                break;
            }

       // }  
         //   std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        
    }

    void UsbCommNode::perceptionTargetCallBack(const std_msgs::msg::Int16::SharedPtr target)
    {

        if((int16_t)(target->data) != 0 && (int16_t)(target->data) < 180 && (int16_t)(target->data) > -180)
            perception_yaw_ = (int16_t)(target->data);
        else
        {
            perception_yaw_ = 0;
        }
        // std::cout << "1:" << perception_yaw_ << std::endl;
        // send_package_._SOF = 0x55;
        // send_package_._EOF = 0xFF;
        // send_package_.ID = RMOS_1_SEND_ID;
        // this->perception_target2state(&(send_package_.AimbotState));
        // send_package_.PitchRelativeAngle = (int16_t)(0);
        // send_package_.YawRelativeAngle = (int16_t)(target->data * 32768.0 / 180.0);
        // // send_package_.SystemTimer = (int16_t)(target->timestamp_recv);
        // transporter_->write((unsigned char *)&send_package_, sizeof(transporter::RMOSSendPackage));
    }

    void UsbCommNode::perception_target2data(const std_msgs::msg::Int16::SharedPtr target, u_char *buf)
    {
        memset(buf, 0, 8);memset(buf, 0, 8);
        int16_t pitch_yaw[2] = {0};
        pitch_yaw[0] = (int16_t)(0);
        pitch_yaw[1] = (int16_t)(target->data * 32768.0 / 180.0);
        memcpy(buf, pitch_yaw, 4);
    }

    void UsbCommNode::perception_target2state(u_char *buf)
    {
        memset(buf, 0, 3);
        buf[0] |= 0x20;
    }
        
} // namespace rmos_comm
#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_transporter_r::UsbCommNode)
