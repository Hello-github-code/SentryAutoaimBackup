//
// Created by Wang on 23-7-18.
//

#ifndef RMOS_BALLISTIC_SOLVER_HPP
#define RMOS_BALLISTIC_SOLVER_HPP

#include "processer_interfaces/ballistic_solver_interface.hpp"
#include <iostream>
#include <vector>

namespace processer
{
    struct BallisticParam
    {
        // 补偿系数
        float left_bs;
        float right_bs;

        double k;   // 空气阻力系数 / 质量
        double g;   // 重力加速度
    };

    class BallisticSolver :  BallisticSolverInterface
    {
    public:
        BallisticSolver();
        ~BallisticSolver();
        cv::Point3f getAngleTimel(cv::Point3f position) override;
        cv::Point3f getAngleTimer(cv::Point3f position) override;
        bool setBulletSpeed(float bullet_speed) override;

        float bullet_speed_ = 20;

    private:
        void setBS_coeff_l(cv::Point3f position);
        void setBS_coeff_r(cv::Point3f position);

        BallisticParam ballistic_param_;
        double bs_coeff;        // 弹速系数
    };
}

#endif //RMOS_BALLISTIC_SOLVER_HPP
