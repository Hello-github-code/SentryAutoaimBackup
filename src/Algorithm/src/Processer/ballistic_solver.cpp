//
// Created by nuc12 on 23-7-18.
//

#include "Processer/ballistic_solver.hpp"

namespace processer
{
    BallisticSolver::BallisticSolver() {
        cv::FileStorage fs("./src/Algorithm/configure/Processer/ballistic_solver/params.xml", cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cout << "open processer ballistic_solver param fail" << std::endl;
            exit(0);
        }

        fs["left_bs"] >> ballistic_param_.left_bs;
        fs["right_bs"] >> ballistic_param_.right_bs;
        fs["k"] >> ballistic_param_.k;
        fs["g"] >> ballistic_param_.g;
        fs["bullet_speed"] >> bullet_speed_;

        fs.release();
    }

    cv::Point3f BallisticSolver::getAngleTimel(cv::Point3f position)
    {
        this->setBS_coeff_l(position);
        double dy, angle, y_actual;
        double t_actual = 0.0;
        double y_temp = position.z / 1000.0 + 0.094;
        double y = y_temp;
        double x = sqrt(position.x * position.x + position.y * position.y) / 1000.0;
        // std::cout << "bsr = " << this->bullet_speed_ <<' '<<this->ballistic_param_.g<< '\n';

        for (int i = 0; i < 40; i++) {
            angle = atan2(y_temp, x);
            // t_actual = (exp(this->ballistic_param_.k * x) - 1.0) /
            //          (this->ballistic_param_.k * this->bs_coeff * this->bullet_speed_ * cos(angle));
            t_actual = x / (this->bullet_speed_ * cos(angle));
            y_actual = double( bullet_speed_ * sin(angle) * t_actual - this->ballistic_param_.g * t_actual * t_actual / 2.0 );
            dy = y - y_actual;
            y_temp += dy;
            if (abs(dy) < 0.001)
                break;
        }

        float pitch = (angle) / M_PI * 180.0;
        if (x < 5)
            pitch -= 0.03 * x + 0.003 * (exp(x) - 1);

        float yaw = atan2(position.y, position.x) / CV_PI * 180.0;

        return cv::Point3f(pitch, yaw, t_actual);
    }
    
    cv::Point3f BallisticSolver::getAngleTimer(cv::Point3f position)
    {
        this->setBS_coeff_r(position);
        double dy, angle, y_actual;
        double t_actual = 0.0;
        double y_temp = position.z / 1000.0 + 0.094;
        double y = y_temp;
        double x = sqrt(position.x * position.x + position.y * position.y) / 1000.0;
        // std::cout << "bsr = " << this->bullet_speed_ <<' '<<this->ballistic_param_.g<< '\n';

        for (int i = 0; i < 40; i++) {
            angle = atan2(y_temp, x);
            // t_actual = (exp(this->ballistic_param_.k * x) - 1.0) /
            //           (this->ballistic_param_.k * this->bs_coeff * this->bullet_speed_ * cos(angle));
            t_actual = x / (this->bullet_speed_ * cos(angle));
            y_actual = double( bullet_speed_ * sin(angle) * t_actual - this->ballistic_param_.g * t_actual * t_actual / 2.0 );
            dy = y - y_actual;
            y_temp += dy;
            if (abs(dy) < 0.001)
                break;
        }

        float pitch = (angle) / M_PI * 180.0;
        if (x < 5)
            pitch -= 0.03 * x + 0.002 * (exp(x) - 1);

        float yaw = atan2(position.y, position.x) / CV_PI * 180.0;

        return cv::Point3f(pitch, yaw, t_actual);
    }

    bool BallisticSolver::setBulletSpeed(float bullet_speed)
    {
        this->bullet_speed_ = bullet_speed;
        return true;
    }

    void BallisticSolver::setBS_coeff_l(cv::Point3f position)
    {
        float distance = sqrt(position.x * position.x +
                              position.y * position.y +
                              position.z * position.z);

        bs_coeff = this->ballistic_param_.left_bs;

        if (distance < 2200 && distance >= 2000) bs_coeff *= 0.82;
        else if (distance < 2000 && distance >= 1800) bs_coeff *= 0.62;
        else if (distance < 1800) bs_coeff *= 0.42;

        if (distance > 6200) bs_coeff *= 0.92;
        if (distance > 6850) bs_coeff *= 1.06;
        if (distance > 3.50) bs_coeff *= 1.025;
        if (distance < 2800) bs_coeff *= 0.94;
        if (distance < 2200) bs_coeff *= 0.94;
        if (distance < 2000) bs_coeff *= 1.1;  
        if (distance < 1800) bs_coeff *= 1.15;       
        if (distance < 1250) bs_coeff *= 0.67;  
        // std::cout << "distance : " << distance << '\n';
    }

    void BallisticSolver::setBS_coeff_r(cv::Point3f position)
    {
        float distance = sqrt(position.x * position.x +
                              position.y * position.y +
                              position.z * position.z);

        bs_coeff= this->ballistic_param_.right_bs;

        if ( distance < 2000) bs_coeff *= 0.92;
        if ( distance > 3000) bs_coeff *= 0.976;
        if ( distance > 6200) bs_coeff *= 0.92;
        if ( distance > 6850) bs_coeff *= 1.06;
    }
}
