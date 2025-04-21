#include "Tool/filter/extend_kalman_filter.hpp"
#include <iostream>

namespace tool
{
    ExtendedKalmanFilter::ExtendedKalmanFilter(
        const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
        const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0)
        : f(f),
          h(h),
          jacobian_f(j_f),
          jacobian_h(j_h),
          update_Q(u_q),
          update_R(u_r),
          P_post(P0),
          n(P0.rows()),
          I(Eigen::MatrixXd::Identity(n, n)),
          x_pri(n),
          x_post(n)
    {}

    void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { this->x_post = x0; }

    Eigen::MatrixXd ExtendedKalmanFilter::predict()
    {
        this->F = jacobian_f(this->x_post), this->Q = update_Q();

        this->x_pri = f(this->x_post);
        this->P_pri = this->F * this->P_post * this->F.transpose() + this->Q;

        // handle the case when there will be no measurement before the next predict
        this->x_post = this->x_pri;
        this->P_post = this->P_pri;

        return this->x_pri;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::predict(double vyaw)
    {
        this->F = jacobian_f(this->x_post), this->Q = update_Q();
        
        if (vyaw > 2)  {
            this->u_q_count++;
        }
        
        if (this->u_q_count % 5 == 0) {
            Q(0,0) /= pow(3, abs(vyaw));
            Q(0,1) /= pow(3, abs(vyaw));
            Q(1,0) /= pow(3, abs(vyaw));
            Q(2,2) /= pow(3, abs(vyaw));
            Q(2,3) /= pow(3, abs(vyaw));
            Q(3,2) /= pow(3, abs(vyaw));

            this->u_q_count = 0;
        }

        this->x_pri = f(this->x_post);
        this->P_pri = this->F * this->P_post * this->F.transpose() + this->Q;

        // handle the case when there will be no measurement before the next predict
        this->x_post = this->x_pri;
        this->P_post = this->P_pri;

        this->last_vx = this->x_pri(1);
        this->last_vy = this->x_pri(3);

        return this->x_pri;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
    {
        this->H = jacobian_h(this->x_pri), this->R = update_R(z);

        this->K = this->P_pri * this->H.transpose() * (this->H * this->P_pri * this->H.transpose() + this->R).inverse();
        this->x_post = this->x_pri + this->K * (z - h(this->x_pri));
        this->P_post = (this->I - this->K * this->H) * this->P_pri;

        return this->x_post;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z, double vyaw)
    {
        this->H = jacobian_h(this->x_pri), this->R = update_R(z);

        double v_xy = sqrt(pow(this->last_vx, 2) + pow(this->last_vy, 2));

        if (v_xy > 1)  {
            this->u_r_count++;
        }

        if (this->u_r_count % 5 == 0) {
            R(0, 0) /= pow(7, v_xy);
            R(1, 1) /= pow(7, v_xy);

            this->u_r_count = 0;
        }

        this->K = this->P_pri * this->H.transpose() * (this->H * this->P_pri * this->H.transpose() + this->R).inverse();
        this->x_post = this->x_pri + this->K * (z - h(this->x_pri));
        this->P_post = (this->I - this->K * this->H) * this->P_pri;

        // dvx = x_post(1) - last_vx;
        // dvy = x_post(3) - last_vy;
        // if (abs(dvx) > 0.8)
        //     x_post(1) = last_vx;
        // if (abs(dvy) > 0.8)
        //     x_post(3) = last_vy;
        // last_vx = x_post(1);
        // last_vy = x_post(3);

        return this->x_post;
    }
}
