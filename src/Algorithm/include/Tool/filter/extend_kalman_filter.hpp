//
// Created by nuc12 on 23-7-12.
//

#ifndef RMOS_EXTEND_KALMAN_FILTER_HPP
#define RMOS_EXTEND_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <functional>

namespace tool
{
    class ExtendedKalmanFilter
    {
    public:
        ExtendedKalmanFilter() = default;

        using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
        using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
        using VoidMatFunc = std::function<Eigen::MatrixXd()>;

        explicit ExtendedKalmanFilter(
                const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
                const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

        // Set the initial state
        void setState(const Eigen::VectorXd & x0);

        // Compute a predicted state
        Eigen::MatrixXd predict(float vyaw);
        Eigen::MatrixXd predict();

        // Update the estimated state based on measurement
        Eigen::MatrixXd update(const Eigen::VectorXd & z);
        Eigen::MatrixXd update(const Eigen::VectorXd & z, double vyaw);

    private:
        // Process nonlinear vector function
        VecVecFunc f;
        // Observation nonlinear vector function
        VecVecFunc h;
        // Jacobian of f()
        VecMatFunc jacobian_f;
        Eigen::MatrixXd F;
        // Jacobian of h()
        VecMatFunc jacobian_h;
        Eigen::MatrixXd H;
        // Process noise covariance matrix
        VoidMatFunc update_Q;
        Eigen::MatrixXd Q;
        // Measurement noise covariance matrix
        VecMatFunc update_R;
        Eigen::MatrixXd R;

        // Priori error estimate covariance matrix
        Eigen::MatrixXd P_pri;
        // Posteriori error estimate covariance matrix
        Eigen::MatrixXd P_post;

        // Kalman gain
        Eigen::MatrixXd K;

        // System dimensions
        int n;

        // N-size identity
        Eigen::MatrixXd I;

        // Priori state
        Eigen::VectorXd x_pri;
        // Posteriori state
        Eigen::VectorXd x_post;
        float last_vx;
        float last_vy;
        float dvx=0;
        float dvy=0;
    };

    class MoveExtendedKalmanFilter
    {
    public:
        MoveExtendedKalmanFilter() = default;

        using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
        using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;
        using VoidMatFunc = std::function<Eigen::MatrixXd()>;

        explicit MoveExtendedKalmanFilter(
                const VecVecFunc & f, const VecVecFunc & h, const VecMatFunc & j_f, const VecMatFunc & j_h,
                const VoidMatFunc & u_q, const VecMatFunc & u_r, const Eigen::MatrixXd & P0);

        // Set the initial state
        void setState(const Eigen::VectorXd & x0);

        // Compute a predicted state
        Eigen::MatrixXd predict();

        // Update the estimated state based on measurement
        Eigen::MatrixXd update(const Eigen::VectorXd & z);

    private:
        // Process nonlinear vector function
        VecVecFunc f;
        // Observation nonlinear vector function
        VecVecFunc h;
        // Jacobian of f()
        VecMatFunc jacobian_f;
        Eigen::MatrixXd F;
        // Jacobian of h()
        VecMatFunc jacobian_h;
        Eigen::MatrixXd H;
        // Process noise covariance matrix
        VoidMatFunc update_Q;
        Eigen::MatrixXd Q;
        // Measurement noise covariance matrix
        VecMatFunc update_R;
        Eigen::MatrixXd R;

        // Priori error estimate covariance matrix
        Eigen::MatrixXd P_pri;
        // Posteriori error estimate covariance matrix
        Eigen::MatrixXd P_post;

        // Kalman gain
        Eigen::MatrixXd K;

        // System dimensions
        int n;

        // N-size identity
        Eigen::MatrixXd I;

        // Priori state
        Eigen::VectorXd x_pri;
        // Posteriori state
        Eigen::VectorXd x_post;
    };
}

#endif //RMOS_EXTEND_KALMAN_FILTER_HPP
