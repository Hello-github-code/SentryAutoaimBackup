//
// Created by nuc12 on 23-7-15.
//

#ifndef RMOS_CONTROLER_HPP
#define RMOS_CONTROLER_HPP

// OpenCV
#include <opencv2/core.hpp>

// Eigen
#include <Eigen/Eigen>

#include "Base/armor.hpp"
#include "Tool/tracker/tracker.hpp"
#include "Processer/ballistic_solver.hpp"

namespace processer
{
    class Controler_l 
    {
    public:
        Controler_l();
        ~Controler_l();

        int getAimingPoint(std::vector<base::Armor> armors, cv::Point3f& aimming_point, double timestamp, int8_t* provirty_list, int* attack_id, cv::Point3f& gun_point);   // 1:move 2:slow_move 3:stop

        bool judgeFire(cv::Point3f aimming_point_camera, double v_yaw, cv::Point2f& aim_point_2d_out, bool print, int id);

        bool getParam(cv::Mat camera_matrix);

        BallisticSolver ballistic_solver_;
        tool::Tracker tracker_[14];

        double gun_pitch_offset_{0};
        double gun_yaw_offset_{0};

    private:
        double dt_[14]{0};
        double lost_time_thres_;
        cv::Mat camera_matrix_;

        double s2q_xyz_{0};
        double s2q_myaw_{0};
        double s2q_ryaw_{0};
        double s2q_a_xy_{0};
        double s2q_r_{0};
        double r_xyz_factor{0};
        double r_myaw{0};
        double r_ryaw{0};
        double r_a_xy{0};
        // cv::Point3d last_v_[14] {cv::Point3d(0,0,0)};

        int true_x_;
        double delay_{0};

        double last_time_[14]{0};
    };

    class Controler_r 
    {
    public:
        Controler_r();
        ~Controler_r();

        int getAimingPoint(std::vector<base::Armor> armors, cv::Point3f& aimming_point, double timestamp, int8_t* provirty_list, int* attack_id, cv::Point3f& gun_point, cv::Point3f& aim2_point, bool center);   // 1:move 2:slow_move 3:stop

        bool judgeFire(cv::Point3f aimming_point_camera, double v_yaw, cv::Point2f& aim_point_2d_out, bool print, int id);

        bool getParam(cv::Mat camera_matrix);

        BallisticSolver ballistic_solver_;
        tool::Tracker tracker_[14];

        double gun_pitch_offset_{0};
        double gun_yaw_offset_{0};

    private:
        double dt_[14]{0};
        double lost_time_thres_;
        cv::Mat camera_matrix_;

        double s2q_xyz_{0};
        double s2q_myaw_{0};
        double s2q_ryaw_{0};
        double s2q_a_xy_{0};
        double s2q_r_{0};
        double r_xyz_factor{0};
        double r_myaw{0};
        double r_ryaw{0};
        double r_a_xy{0};

        int true_x_;
        double delay_{0};
        double o_delay{0};

        double last_time_[14]{0};
    };
}

#endif //RMOS_CONTROLER_HPP
