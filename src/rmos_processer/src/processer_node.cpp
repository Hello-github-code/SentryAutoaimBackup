//
// Created by nuc12 on 23-7-10.
//

#include "../include/processer_node.hpp"
#include "angles/angles.h"

namespace rmos_processer {
    ProcesserNode::ProcesserNode(const rclcpp::NodeOptions &options)
        : Node("processer", options)
    {
        // 从参数获取 is_left
        this->declare_parameter("is_left", false);
        is_left_ = this->get_parameter("is_left").as_bool();

        std::string node_name = is_left_ ? "processer_l" : "processer_r";
        RCLCPP_INFO(this->get_logger(), "Starting %s node", node_name.c_str());

        controler_ = std::make_shared<processer::Controler>(is_left_);

        std::string camera_topic = is_left_ ? "/daheng_camera_info_l" : "/daheng_camera_info_r";
        std::string armor_topic = is_left_ ? "/rmos_detector/armors_l" : "/rmos_detector/armors_r";
        std::string target_topic = is_left_ ? "/target_l" : "/target_r";
        std::string aim_topic = is_left_ ? "/aim_l" : "/aim_r";
        std::string follow_topic = is_left_ ? "/follow_target_l" : "/follow_target_r";
        std::string yaw_topic = is_left_ ? "/yawl_marker" : "/yawr_marker";
        std::string service_name = is_left_ ? "AimTarget_l" : "AimTarget_r";
        std::string autoaim_topic = is_left_ ? "/autoaim_state_l" : "/autoaim_state_r";
        std::string marker_topic = is_left_ ? "/process/marker_l" : "/process/marker_r";
        std::string detect_topic = is_left_ ? "/detect/marker_l" : "/detect/marker_r";

        this->camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_msg)
            {
                RCLCPP_INFO(this->get_logger(), "Receive camera infomation in processer");

                this->camera_info_msg_ = *camera_info_msg;

                this->camera_matrix_.create(3, 3, CV_64FC1);

                for (int i = 0; i < 9; i++)
                {
                    this->camera_matrix_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
                }
                this->camera_info_sub_.reset();
            });

        this->prosser_server_ =this->create_service<sentry_interfaces::srv::AimTarget>(
            service_name, 
            std::bind(&ProcesserNode::sentrycallback, this, std::placeholders::_1, std::placeholders::_2));

        // set subscriber for imu_time, armor, and bullet_speed
        using rclcpp::CallbackGroupType;
        this->armors_sub_callback_group_ = this->create_callback_group(CallbackGroupType::Reentrant);
        armors_sub_.subscribe(this, armor_topic, rmw_qos_profile_sensor_data);

        // Subscriber with tf2 message_filter
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
                this->get_node_base_interface(), this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
        target_frame_ = "Gimbal_Link";
        tf2_filter_ = std::make_shared<tf2_filter>(
                armors_sub_, *tf2_buffer_, target_frame_, 100, this->get_node_logging_interface(),
                this->get_node_clock_interface(), std::chrono::duration<int>(100));
        tf2_filter_->registerCallback(&ProcesserNode::armorsCallBack, this);

        bs_buff.push(22);
        bs_total += 22;

        this->bs_sub_callback_group_ = this->create_callback_group(CallbackGroupType::Reentrant);
        auto bs_sub_option = rclcpp::SubscriptionOptions();
        bs_sub_option.callback_group = this->bs_sub_callback_group_;
        this->bs_sub_ = this->create_subscription<rm2_referee_msgs::msg::ShootData>(
            "/rm2_referee/shoot_data", rclcpp::SensorDataQoS(),
            std::bind(&ProcesserNode::bsCallBack, this, std::placeholders::_1),
            bs_sub_option);
        
        this->aimstate_sub_callback_group_ = this->create_callback_group(CallbackGroupType::Reentrant);
        auto autoaim_state_sub_option = rclcpp::SubscriptionOptions();
        autoaim_state_sub_option.callback_group = this->aimstate_sub_callback_group_;
        this->autoaim_state_sub_ = this->create_subscription<rmos_interfaces::msg::AutoaimState>(
            autoaim_topic, rclcpp::SensorDataQoS(),
            std::bind(&ProcesserNode::autoaimStateCallBack, this, std::placeholders::_1),
            autoaim_state_sub_option);

        // set publisher
        this->target_pub_ = this->create_publisher<rmos_interfaces::msg::Target>(
            target_topic, rclcpp::SensorDataQoS());
        this->sentry_target_pub_ = this->create_publisher<sentry_interfaces::msg::FollowTarget>(
            follow_topic, 1);
        this->yaw_mark_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            yaw_topic, rclcpp::SensorDataQoS());
        this->aim_pub_ = this->create_publisher<rmos_interfaces::msg::Aimpoint>(
            aim_topic, rclcpp::SensorDataQoS());

        armor_marker_.ns = "armors_cube";
        armor_marker_.header.frame_id = "Gimbal_Link";
        armor_marker_.action = visualization_msgs::msg::Marker::ADD;
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.02;
        armor_marker_.scale.y = 0.125;
        armor_marker_.scale.z = 0.08;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.r = 1.0;
        armor_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        text_marker_.ns = "classification";
        text_marker_.header.frame_id = "Gimbal_Link";
        text_marker_.action = visualization_msgs::msg::Marker::ADD;
        text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker_.scale.z = 0.1;
        text_marker_.color.a = 1.0;
        text_marker_.color.r = 1.0;
        text_marker_.color.g = 1.0;
        text_marker_.color.b = 1.0;
        text_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

        position_marker_.ns = "position";
        position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
        position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
        position_marker_.color.a = 1.0;
        position_marker_.color.g = 1.0;
        linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
        linear_v_marker_.ns = "linear_v";
        linear_v_marker_.scale.x = 0.03;
        linear_v_marker_.scale.y = 0.05;
        linear_v_marker_.color.a = 1.0;
        linear_v_marker_.color.r = 1.0;
        linear_v_marker_.color.g = 1.0;
        angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
        angular_v_marker_.ns = "angular_v";
        angular_v_marker_.scale.x = 0.03;
        angular_v_marker_.scale.y = 0.05;
        angular_v_marker_.color.a = 1.0;
        angular_v_marker_.color.b = 1.0;
        angular_v_marker_.color.g = 1.0;
        armors_marker_.ns = "armors";
        armors_marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        armors_marker_.scale.x = armors_marker_.scale.y = armors_marker_.scale.z = 0.1;
        armors_marker_.color.a = 1.0;
        armors_marker_.color.r = 1.0;
        aiming_marker_.ns = "aiming_point";
        aiming_marker_.type = visualization_msgs::msg::Marker::SPHERE;
        aiming_marker_.scale.x = aiming_marker_.scale.y = aiming_marker_.scale.z = 0.1;
        aiming_marker_.color.a = 1.0;
        aiming_marker_.color.b = 1.0;

        this->detect_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            detect_topic, 10);
        this->process_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            marker_topic, 10);

        small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, -small_height / 2.0));
        small_armor.push_back(cv::Point3f(0.0,small_width / 2.0, small_height / 2.0));
        small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, small_height / 2.0));
        small_armor.push_back(cv::Point3f(0.0,-small_width / 2.0, -small_height / 2.0));

        big_armor.push_back(cv::Point3d(0.0,big_width / 2.0, -big_height / 2.0));
        big_armor.push_back(cv::Point3d(0.0,big_width / 2.0,  big_height / 2.0));
        big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, big_height / 2.0));
        big_armor.push_back(cv::Point3d(0.0,-big_width / 2.0, -big_height / 2.0));

        // 根据 is_left_ 选择对应的坐标系
        std::string cam_link = is_left_ ? "LCam_Link" : "RCam_Link";
        
        // 修改所有 marker 的 frame_id
        armor_marker_.header.frame_id = cam_link;
        text_marker_.header.frame_id = cam_link;
        position_marker_.header.frame_id = cam_link;
        linear_v_marker_.header.frame_id = cam_link;
        angular_v_marker_.header.frame_id = cam_link;
        armors_marker_.header.frame_id = cam_link;
        aiming_marker_.header.frame_id = cam_link;
    }

    void ProcesserNode::armorsCallBack(const rmos_interfaces::msg::Armors::SharedPtr armors_msg)
    {
        this->controler_->getParam(this->camera_matrix_);

        rmos_interfaces::msg::Aimpoint aim_msg;

        rmos_interfaces::msg::Target target_msg;
        target_msg.header.frame_id = target_frame_;

        detect_marker_array_.markers.clear();
        process_marker_array_.markers.clear();
        armor_marker_.points.clear();
        armor_marker_.id = 0;
        text_marker_.id = 0;

        sentry_interfaces::msg::FollowTarget followtarget_msg;
        followtarget_msg.target.header.stamp = armors_msg->header.stamp;

        std::vector <base::Armor> new_armors;

        double timestamp = armors_msg->header.stamp.sec + armors_msg->header.stamp.nanosec * 1e-9;

        // 根据 is_left_ 选择对应的坐标系
        std::string cam_link = is_left_ ? "LCam_Link" : "RCam_Link";

        for (auto &armor: armors_msg->armors) {
            if (attack_id == armor.num_id && armor.confidence < 80 && armor.num_id != 7)
                continue;

            base::Armor new_armor;
            new_armor.num_id = armor.num_id;
            new_armor.k_ = armor.k;
            new_armor.leftgap = armor.gap[0];
            new_armor.rightgap = armor.gap[1];

            geometry_msgs::msg::PoseStamped ps;
            ps.header = armors_msg->header;
            ps.pose = armor.pose;
            try {
                armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
            }
            catch (const tf2::LookupException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }
            catch (const tf2::ExtrapolationException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }
            // std::cout << "confidence " << (int)(armor.confidence) << '\n';

            // Get armor yaw
            tf2::Quaternion tf_armor_q;
            tf2::fromMsg(armor.pose.orientation, tf_armor_q);
            double armor_roll, armor_pitch, armor_yaw;
            tf2::Matrix3x3(tf_armor_q).getRPY(armor_roll, armor_pitch, armor_yaw);

            double temp_roll = 0;
            double temp_pitch;
            double temp_yaw = armor_yaw;

            temp_pitch = -15.0 / 180.0 * M_PI;
            if (armor.num_id == 7)
                temp_pitch = 15.0 / 180.0 * M_PI;

            geometry_msgs::msg::PoseStamped guess_pose;
            guess_pose.header = armors_msg->header;
            guess_pose.pose.orientation.x = sin(temp_pitch/2) * cos(temp_roll/2) * cos(temp_yaw/2) - cos(temp_pitch/2) * sin(temp_roll/2) * sin(temp_yaw/2);
            guess_pose.pose.orientation.y = cos(temp_pitch/2) * sin(temp_roll/2) * cos(temp_yaw/2) + sin(temp_pitch/2) * cos(temp_roll/2) * sin(temp_yaw/2);
            guess_pose.pose.orientation.z = cos(temp_pitch/2) * cos(temp_roll/2) * sin(temp_yaw/2) - sin(temp_pitch/2) * sin(temp_roll/2) * cos(temp_yaw/2);
            guess_pose.pose.orientation.w = cos(temp_pitch/2) * cos(temp_roll/2) * cos(temp_yaw/2) + sin(temp_pitch/2) * sin(temp_roll/2) * sin(temp_yaw/2);
            geometry_msgs::msg::PoseStamped gs;
            gs = guess_pose;
            gs.header.frame_id = target_frame_;
            try {
                guess_pose.pose = tf2_buffer_->transform(gs, cam_link).pose;
            }
            catch (const tf2::LookupException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }
            catch (const tf2::ExtrapolationException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }

            double guess_theta = 2 * acos(guess_pose.pose.orientation.w);

            cv::Mat correct_rvec,correct_tvec;
            correct_rvec.create(3, 1, CV_64FC1);
            correct_tvec.create(3, 1, CV_64FC1);

            correct_rvec.at<double>(0,0) = guess_pose.pose.orientation.x / sin(guess_theta*0.5) * guess_theta;
            correct_rvec.at<double>(1,0) = guess_pose.pose.orientation.y / sin(guess_theta*0.5) * guess_theta;
            correct_rvec.at<double>(2,0) = guess_pose.pose.orientation.z / sin(guess_theta*0.5) * guess_theta;

            correct_tvec.at<double>(0,0) = armor.pose.position.x * 1000;
            correct_tvec.at<double>(1,0) = armor.pose.position.y * 1000;
            correct_tvec.at<double>(2,0) = armor.pose.position.z * 1000;

            std::vector<cv::Point2d> image_points;
            cv::Point2d image_point;
            cv::Point2d center_point;

            for (int i = 0; i < 4; i++) {
                image_point.x = armor.points[2*i];
                image_point.y = armor.points[2*i+1];
                image_points.push_back(image_point);
                new_armor.points.push_back(image_point);
                center_point.x += image_point.x;
                center_point.y += image_point.y;
            }
            center_point.x = center_point.x / 4;
            center_point.y = center_point.y / 4;
            new_armor.center_point = center_point;
            std::vector<cv::Point2d> point2D;
            point2D = image_points;

            switch (armor.type)
            {
                case false:
                    cv::solvePnP(small_armor, point2D, camera_matrix_, dist_coeffs_, correct_rvec, correct_tvec, true, cv::SOLVEPNP_IPPE);
                    break;
                case true:
                    cv::solvePnP(big_armor, point2D, camera_matrix_, dist_coeffs_, correct_rvec, correct_tvec, true, cv::SOLVEPNP_IPPE);
                    break;
                default:
                    cv::solvePnP(big_armor, point2D, camera_matrix_, dist_coeffs_, correct_rvec, correct_tvec, true, cv::SOLVEPNP_IPPE);
                    break;
            }

            cv::Mat rotation_matrix;
            cv::Rodrigues(correct_rvec, rotation_matrix);
                tf2::Matrix3x3 tf2_rotation_matrix(
                rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1),
                rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(1, 0),
                rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
                rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1),
                rotation_matrix.at<double>(2, 2));
            
            tf2::Quaternion tf2_quaternion;
            tf2_rotation_matrix.getRotation(tf2_quaternion);
            armor.pose.orientation.x = tf2_quaternion.x();
            armor.pose.orientation.y = tf2_quaternion.y();
            armor.pose.orientation.z = tf2_quaternion.z();
            armor.pose.orientation.w = tf2_quaternion.w();

            armor.pose.position.x = correct_tvec.at<double>(0, 0) / 1000;
            armor.pose.position.y = correct_tvec.at<double>(1, 0) / 1000;
            armor.pose.position.z = correct_tvec.at<double>(2, 0) / 1000;

            ps.pose = armor.pose; 
            try {
                armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
            }                
            catch (const tf2::LookupException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }
            catch (const tf2::ExtrapolationException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }

            new_armor.position.x = armor.pose.position.x;
            new_armor.position.y = armor.pose.position.y;
            new_armor.position.z = armor.pose.position.z;

            tf2::fromMsg(armor.pose.orientation, tf_armor_q);

            tf2::Matrix3x3(tf_armor_q).getRPY(armor_roll, armor_pitch, armor_yaw);
            armor_yaw=armor.yaw;
            std_msgs::msg::Float32 yaw_msgs;
            yaw_msgs.data = armor_yaw/3.14159*180;
            yaw_mark_pub_->publish(yaw_msgs);
            // std::cout << "armor_yaw = " << armor_yaw / 3.1415926 * 180 << "armor_pitch = " << armor_pitch / 3.1415926 * 180 << "   armor_roll = " << armor_roll / 3.1415926 * 180 << std::endl;

            new_armor.yaw = controler_->tracker_[armor.num_id].last_yaw_ +
                angles::shortest_angular_distance(controler_->tracker_[armor.num_id].last_yaw_, armor_yaw);

            if (new_armor.position.z > 0.5 && new_armor.num_id != 7)
                continue;

            cv::Point3f armor_point;
            armor_point.x = new_armor.position.x;
            armor_point.y = new_armor.position.y;
            armor_point.z = new_armor.position.z;
            if (controler_->ballistic_solver_.getAngleTimer(armor_point * 1000).x > 13 && armor.num_id != 7)
                continue;

            // erase armors ...
            new_armors.push_back(new_armor);

            armor_marker_.id++;
            armor_marker_.pose = armor.pose;
            text_marker_.id++;
            text_marker_.pose.position = armor.pose.position;
            text_marker_.pose.position.y -= 0.1;
            text_marker_.text = std::to_string(armor.num_id);
            detect_marker_array_.markers.emplace_back(text_marker_);
        }

        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        auto time = steady_clock_.now();
        if (time.seconds() - sentry_last_time_ > 5.0) {
            int p[14] = {7, 6, 1, 3, 11, 4, 12, 2, 5, 13, -1, -1, -1, -1};
            for (int i = 0; i < 14; i++)
            prioritys[i] = p[i];
            mode_ = 0;
        }

        target_msg.mode = this->mode_;

        cv::Point3f aiming_point;
        cv::Point3f aiming2_point;
        cv::Point3f gun_point;
        bool if_center = false;
        int move_state = controler_->getAimingPoint(new_armors, aiming_point, timestamp, prioritys, &attack_id, gun_point, aiming2_point, if_center);

        cv::Point3f car_point;
        float x0, y0, vx, vy, dt;

        if (attack_id > 0) {
            car_point.x = controler_->tracker_[attack_id].move_target_state[0];
            car_point.y = controler_->tracker_[attack_id].move_target_state[2];
            car_point.z = controler_->tracker_[attack_id].move_target_state[4];
            x0 = controler_->tracker_[attack_id].move_target_state[0];
            y0 = controler_->tracker_[attack_id].move_target_state[2];
            vx = controler_->tracker_[attack_id].move_target_state[1];
            vy = controler_->tracker_[attack_id].move_target_state[3];
            dt = controler_->ballistic_solver_.getAngleTimer(car_point * 1000).z;
        }

        int count_id = 1;
        for (int i = 0; i < 14; i++) {
            if (attack_id == (int)prioritys[i])
                break;
            count_id++;
        }

        sentry_transform = tf2_buffer_->lookupTransform("Gimbal_Link", cam_link, armors_msg->header.stamp);
        tf2::Quaternion sentry_tf_q;
        tf2::fromMsg(sentry_transform.transform.rotation, sentry_tf_q);
        double tf_roll, tf_pitch, tf_yaw;
        tf2::Matrix3x3(sentry_tf_q).getRPY(tf_roll, tf_pitch, tf_yaw);

        if ((attack_id >= 0) &&
            (controler_->tracker_[attack_id].tracker_state == base::TRACKING
           ||controler_->tracker_[attack_id].tracker_state == base::TEMP_LOST
           ||controler_->tracker_[attack_id].tracker_state == base::DETECTING))
        {
            followtarget_msg.target.point.x = -sin(tf_yaw) * (x0+vx*dt) + cos(tf_yaw) * (y0+vy*dt);
            followtarget_msg.target.point.y = -cos(tf_yaw) * (x0+vx*dt) - sin(tf_yaw) * (y0+vy*dt);
            followtarget_msg.target.point.z = controler_->tracker_[attack_id].move_target_state[4];
            followtarget_msg.have_target = 0;
            if (controler_->tracker_[attack_id].tracker_state == base::TRACKING)
                followtarget_msg.have_target = 1;
            else if (controler_->tracker_[attack_id].tracker_state == base::TEMP_LOST)
                followtarget_msg.have_target = 2;
            followtarget_msg.priority = count_id;
        }
        else{
            followtarget_msg.target.point.x = 0;
            followtarget_msg.target.point.y = 0;
            followtarget_msg.target.point.z = 0;
            followtarget_msg.have_target = 0;
            followtarget_msg.priority = prioritys[0];
        }
        sentry_target_pub_->publish(followtarget_msg);

        if (move_state != 3)
        {
            target_msg.id = this->attack_id;
            target_msg.track_state = this->controler_->tracker_[attack_id].tracker_state;
            target_msg.position.x = aiming_point.x;
            target_msg.position.y = aiming_point.y;
            target_msg.position.z = aiming_point.z;
            target_msg.v_yaw = this->controler_->tracker_[attack_id].rotate_target_state(1);
            if (controler_->tracker_[attack_id].rotate_target_state(1) > 0) {
                target_msg.outpost_direction = 1;
            } else {
                target_msg.outpost_direction = -1;
            }

            // 计算 gun_pitch 和 gun_yaw
            cv::Point3f p_y_t = controler_->ballistic_solver_.getAngleTimel(aiming_point * 1000);
            float new_pitch = p_y_t.x;
            // float new_yaw = p_y_t.y;
            float gun_move_yaw = controler_->ballistic_solver_.getAngleTimer(gun_point * 1000).y;

            float delta_pitch = -new_pitch;
            // float delta_yaw = new_yaw;

            // float gun_pitch_offset = -14.951;
            // if (aiming_point.z > 0.1 && attack_id != 7)
            //     gun_pitch_offset = -2.681;
            // if (attack_id == 7)
            //     gun_pitch_offset = -1.789;

            // float gun_yaw_offset = 0.96;
            // if (attack_id != 7)
            //     gun_yaw_offset += 0.2;

            double gun_pitch_offset_ = controler_->gun_pitch_offset_;
            double gun_yaw_offset_ = controler_->gun_yaw_offset_;
            float gun_pitch = delta_pitch + gun_pitch_offset_;
            float gun_yaw = gun_move_yaw + gun_yaw_offset_;

            // 将瞄准点投影回2d平面，通过像素距离判断，判断开火
            bool is2_fire = false;
            if (if_center)
            {
                geometry_msgs::msg::PoseStamped p2x;
                p2x.header = target_msg.header;
                p2x.pose.position.x = aiming2_point.x * 1000;
                p2x.pose.position.y = aiming2_point.y * 1000;
                p2x.pose.position.z = aiming2_point.z * 1000;
                try {
                    p2x.pose = tf2_buffer_->transform(p2x, cam_link).pose;
                } catch (const tf2::LookupException &ex) {
                    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                    return;
                } catch (const tf2::ExtrapolationException &ex) {
                    RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                    return;
                }
                cv::Point3f aiming2_point_camera(p2x.pose.position.x, p2x.pose.position.y, p2x.pose.position.z);

                cv::Point2f aim2_point_2d;
                is2_fire = this->controler_->judgeFire(aiming2_point_camera, this->controler_->tracker_[attack_id].rotate_target_state(1), aim2_point_2d, false, attack_id);
            }

            geometry_msgs::msg::PoseStamped px;
            px.header = target_msg.header;
            px.pose.position.x = aiming_point.x * 1000;
            px.pose.position.y = aiming_point.y * 1000;
            px.pose.position.z = aiming_point.z * 1000;
            try {
                px.pose = tf2_buffer_->transform(px, cam_link).pose;
            } catch (const tf2::LookupException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            } catch (const tf2::ExtrapolationException &ex) {
                RCLCPP_ERROR(get_logger(), "Error while transforming %s", ex.what());
                return;
            }
            cv::Point3f aiming_point_camera(px.pose.position.x, px.pose.position.y, px.pose.position.z);

            cv::Point2f aim_point_2d;
            bool is_fire = this->controler_->judgeFire(aiming_point_camera, this->controler_->tracker_[attack_id].rotate_target_state(1), aim_point_2d, true, attack_id);

            aim_msg.aim_point.x = aim_point_2d.x;
            aim_msg.aim_point.y = aim_point_2d.y;
            this->aim_pub_->publish(aim_msg);

            // if (if_center)
            //     is_fire |= is2_fire;
            // is_fire = is2_fire;
            // if (attack_id == 7 && abs(this->controler_->tracker_[attack_id].rotate_target_state(1)) != 2.512)
            //     is_fire = false;
            // if (move_state == 2)
            //     is_fire = false;
            // if (abs(target_msg.position.x * target_msg.position.x + target_msg.position.y * target_msg.position.y) > 45 && attack_id != 7)
            //     is_fire = false;
            // if (gun_pitch > 15)
            //     is_fire = false;
            
            target_msg.suggest_fire = is_fire;

            if (move_state == 1) {
                target_msg.gun_pitch = gun_pitch;
                target_msg.gun_yaw = gun_yaw;
                // if (attack_id == 7 && target_msg.gun_pitch < -13.4)
                //     target_msg.gun_pitch += 1.3;
            } else {
                target_msg.gun_pitch = gun_pitch;
                target_msg.gun_yaw = gun_yaw;
                // if (attack_id == 7 && target_msg.gun_pitch < -13.4)
                //     target_msg.gun_pitch += 1.3;
            }

            if (gun_pitch < 0) {
                target_msg.id = -1;
                target_msg.track_state = this->controler_->tracker_[9].tracker_state;
                target_msg.position.x = 0;
                target_msg.position.y = 0;
                target_msg.position.z = 0;
                target_msg.gun_pitch = 0;
                target_msg.gun_yaw = 0;
            }
        }
        else
        {
            target_msg.id = -1;
            target_msg.track_state = this->controler_->tracker_[9].tracker_state;
            target_msg.position.x = 0;
            target_msg.position.y = 0;
            target_msg.position.z = 0;
            target_msg.gun_pitch = 0;
            target_msg.gun_yaw = 0;
        }

        target_pub_->publish(target_msg);

        if (debug::get_debug_option(base::SHOW_RVIZ))
        {
            using Marker = visualization_msgs::msg::Marker;
            armor_marker_.action = (armors_msg->armors).empty() ? Marker::DELETE : Marker::ADD;
            detect_marker_array_.markers.emplace_back(armor_marker_);
            publishMarkers(target_msg);
        }
        detect_marker_pub_->publish(detect_marker_array_);
    }

    void ProcesserNode::bsCallBack(const rm2_referee_msgs::msg::ShootData::SharedPtr bs_msg)
    {
        if (is_left_ && (*bs_msg).shooter_number == 1) {
            float bullet_speed = (*bs_msg).initial_speed;

            if (bs_buff.empty() && bullet_speed > 20) {
                this->bs_buff.push(bullet_speed);
                bs_total += bullet_speed;
            }
            else if (bullet_speed != bs_buff.back() && bullet_speed > 20) {
                bs_buff.push(bullet_speed);
                bs_total += bullet_speed;
            }

            if (bs_buff.size() > 20) {
                bs_total -= bs_buff.front();
                bs_buff.pop();
            }

            if (!bs_buff.empty())
                bs_v = bs_total / bs_buff.size();

            std::cout << "Left ====== BulletSpeed ======= " << bs_v << '\n';
            this->controler_->ballistic_solver_.setBulletSpeed(bs_v);
        } else if (!is_left_ && (*bs_msg).shooter_number == 2) {
            float bullet_speed = (*bs_msg).initial_speed;

            if (bs_buff.empty() && bullet_speed > 20) {
                this->bs_buff.push(bullet_speed);
                bs_total += bullet_speed;
            }
            else if (bullet_speed != bs_buff.back() && bullet_speed > 20) {
                bs_buff.push(bullet_speed);
                bs_total += bullet_speed;
            }

            if (bs_buff.size() > 20) {
                bs_total -= bs_buff.front();
                bs_buff.pop();
            }

            if (!bs_buff.empty())
                bs_v = bs_total / bs_buff.size();

            std::cout << "Right ====== BulletSpeed ======= " << bs_v << '\n';
            this->controler_->ballistic_solver_.setBulletSpeed(bs_v);
        }
    }

    void ProcesserNode::autoaimStateCallBack(const rmos_interfaces::msg::AutoaimState::SharedPtr autoaim_state_msg)
    {
        if (this->autoaim_state_buf_.size() > 0) {
            if (autoaim_state_buf_.back().autoaim_state == 1 && (*autoaim_state_msg).autoaim_state == 0) {
                if (attack_id >= 0)
                    this->controler_->tracker_[attack_id].reset();
            }
        }

        this->autoaim_state_buf_.push(*autoaim_state_msg);
        if (this->autoaim_state_buf_.size() > 5) {
            this->autoaim_state_buf_.pop();
        }
    }

    void ProcesserNode::publishMarkers(const rmos_interfaces::msg::Target &target_msg)
    {
        position_marker_.header = target_msg.header;
        linear_v_marker_.header = target_msg.header;
        angular_v_marker_.header = target_msg.header;
        armors_marker_.header = target_msg.header;
        aiming_marker_.header = target_msg.header;
        
        // get armors num
        int armors_num = 4;
        if (attack_id == 11 || attack_id == 12 || attack_id == 13) {
            armors_num = 2;
        } else if (attack_id == 7) {
            armors_num = 3;
        }

        bool is_tracking = false;
        if (controler_->tracker_[attack_id].tracker_state == base::TEMP_LOST || controler_->tracker_[attack_id].tracker_state  == base::TRACKING)
            is_tracking = true;

        if (is_tracking) {
            double yaw = controler_->tracker_[attack_id].rotate_target_state(0), r1 = controler_->tracker_[attack_id].move_target_state(7), r2 = controler_->tracker_[attack_id].another_r;
            double xc = controler_->tracker_[attack_id].move_target_state(0), yc = controler_->tracker_[attack_id].move_target_state(2), za = controler_->tracker_[attack_id].move_target_state(4);
            double vx = controler_->tracker_[attack_id].move_target_state(1), vy = controler_->tracker_[attack_id].move_target_state(3), vz = controler_->tracker_[attack_id].move_target_state(5);
            double dz = controler_->tracker_[attack_id].dz;
            // std::cout << "dz = " << dz << '\n';
            double v_yaw =  controler_->tracker_[attack_id].rotate_target_state(1);
            position_marker_.action = visualization_msgs::msg::Marker::ADD;
            position_marker_.pose.position.x = xc;
            position_marker_.pose.position.y = yc;
            position_marker_.pose.position.z = za + dz / 2;

            linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            linear_v_marker_.points.clear();
            linear_v_marker_.points.emplace_back(position_marker_.pose.position);
            geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
            arrow_end.x += vx;
            arrow_end.y += vy;
            arrow_end.z += vz;
            linear_v_marker_.points.emplace_back(arrow_end);

            angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
            angular_v_marker_.points.clear();
            angular_v_marker_.points.emplace_back(position_marker_.pose.position);
            arrow_end = position_marker_.pose.position;
            arrow_end.z += v_yaw / M_PI;
            angular_v_marker_.points.emplace_back(arrow_end);

            armors_marker_.action = visualization_msgs::msg::Marker::ADD;
            armors_marker_.points.clear();
            // Draw armors
            bool is_current_pair = true;
            size_t a_n = armors_num;
            geometry_msgs::msg::Point p_a;
            double r = 0;
            for (size_t i = 0; i < a_n; i++) {
                double tmp_yaw = yaw + i * (2 * M_PI / a_n);
                // Only 4 armors has 2 radius and height
                if (a_n == 4) {
                    r = is_current_pair ? r1 : r2;
                    p_a.z = za + (is_current_pair ? 0 : dz);
                    is_current_pair = !is_current_pair;
                } else {
                    r = r1;
                    p_a.z = za;
                }
                p_a.x = xc - r * cos(tmp_yaw);
                p_a.y = yc - r * sin(tmp_yaw);
                armors_marker_.points.emplace_back(p_a);
            }
            aiming_marker_.action = visualization_msgs::msg::Marker::ADD;
            aiming_marker_.pose.position.x = target_msg.position.x;
            aiming_marker_.pose.position.y = target_msg.position.y;
            aiming_marker_.pose.position.z = target_msg.position.z;
        } else {
            position_marker_.action = visualization_msgs::msg::Marker::DELETE;
            linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
            angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
            armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
        }

        process_marker_array_.markers.emplace_back(position_marker_);
        process_marker_array_.markers.emplace_back(linear_v_marker_);
        process_marker_array_.markers.emplace_back(angular_v_marker_);
        process_marker_array_.markers.emplace_back(armors_marker_);
        process_marker_array_.markers.emplace_back(aiming_marker_);

        process_marker_pub_->publish(process_marker_array_);
    }

    void ProcesserNode::sentrycallback(const std::shared_ptr<sentry_interfaces::srv::AimTarget::Request> request,
        std::shared_ptr<sentry_interfaces::srv::AimTarget::Response> response)
    {
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        auto time = steady_clock_.now();
        sentry_last_time_ = time.seconds();
       
        int i = 0;
        for (const auto &list_ : request->list)
        {
            if (list_ > 8)
                continue;

            prioritys[i] = list_;
            i++;

            if (list_ == 3) {
                prioritys[i] = 11;
                i++;
            }
            if (list_ == 4) {
                prioritys[i] = 12;
                i++;
            }
            if (list_ == 5) {
                prioritys[i] = 13;
                i++;
            }    
        }

        if (request->list.front() == 7)
            this->mode_ = 1;
        else if (request->list.back() == 255)
            this->mode_ = 2;
        else
            this->mode_ = 0;

        for (i; i < 14; i++)
            prioritys[i] = -1;

        response->success=true;
    }
}

#include <rclcpp_components/register_node_macro.hpp>

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rmos_processer::ProcesserNode)
