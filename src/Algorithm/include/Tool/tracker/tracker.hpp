//
// Created by nuc12 on 23-7-12.
//

#ifndef RMOS_TRACKER_HPP
#define RMOS_TRACKER_HPP

// Eigen
#include <Eigen/Eigen>

// Opencv
#include <opencv2/core.hpp>

// STD
#include <memory>
#include <iostream>

#include "Base/armor.hpp"
#include "Tool/filter/extend_kalman_filter.hpp"

namespace tool
{
    class Tracker
    {
    public:
        Tracker();
        ~Tracker();
        void init(std::vector<base::Armor> armors);
        void update(std::vector<base::Armor> armors);
        void reset();

        base::TrackState tracker_state;
        ExtendedKalmanFilter ekf;

        ExtendedKalmanFilter moveekf;
        ExtendedKalmanFilter rotateekf;
        ExtendedKalmanFilter accelerateekf;
        Eigen::VectorXd move_measurement;
        Eigen::VectorXd rotate_measurement;
        Eigen::VectorXd accelerate_measurement;
        Eigen::VectorXd move_target_state;
        Eigen::VectorXd rotate_target_state;
        Eigen::VectorXd accelerate_target_state;

        int tracked_id;
        int tracking_thres;
        double lost_thres;

        // To store another pair of armors message
        double dz, another_r;
        double last_yaw_{0};
        int lost_count_{0};
        int vyaw_over_count{0};

    private:
        base::Armor tracked_armor;
        std::vector<bool>last_obstate;
        int outpost_direction{0};
        double max_match_distance_;
        double max_match_yaw_diff_;
        int detect_count_;
        int outpost_direction_{1};
        int last_armor_count{0};
        double last_center_x;
        int outpout_direction_count{0};
        int last_state{-1};
        Eigen::Vector3d last_p;
        int outpost_x;
        int outpost_y;

        void initEKF(const base::Armor & a);
        void handleArmorJump(const base::Armor & a);
        Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & mx, const Eigen::VectorXd & rx);
    };
}

#endif //RMOS_TRACKER_HPP
