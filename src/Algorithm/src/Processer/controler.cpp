//
// Created by Wang on 23-7-16.
//

#include "Processer/controler.hpp"

namespace processer
{
    Controler_l::Controler_l()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Processer/controler/param_l.xml", cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cout << "open processer controler param fail" << std::endl;
            exit(0);
        }

        fs["lost_time_thres"] >> lost_time_thres_;
        fs["s2q_xyz"] >> s2q_xyz_;
        fs["s2q_myaw"] >> s2q_myaw_;
        fs["s2q_ryaw"] >> s2q_ryaw_;
        fs["s2q_a_xy"] >> s2q_a_xy_;

        fs["s2q_r"] >> s2q_r_;

        fs["r_xyz_factor"] >> r_xyz_factor;
        fs["r_myaw"] >> r_myaw;
        fs["r_ryaw"] >> r_ryaw;
        fs["r_a_xy"] >> r_a_xy;
        
        fs["delay"] >> delay_;
        fs["true_x"] >> true_x_;

        fs["gun_pitch_offset"] >> this->gun_pitch_offset_;
        fs["gun_yaw_offset"] >> this->gun_yaw_offset_;

        fs.release();

        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, v_xc, yc, v_yc, za, v_za, r
        // measurement: xa, ya, za, yaw

        // move ekf
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, r 
        // measurement: xa, ya, za, yaw 

        // rotate ekf
        // state: yaw, vyaw 
        // measurement: yaw

        // accelerate ekf
        // state: v_xc, a_xc, v_yc, a_yc
        // measurement: v_xc, v_yc

/////////////////////////////   0
    {
        auto fm0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            x_new(2) += x(3) * dt_[0];
            x_new(4) += x(5) * dt_[0];
            return x_new;
        };
        auto fr0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            return x_new;
        };
        auto fa0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            x_new(2) += x(3) * dt_[0];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[0], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[0], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[0], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[0], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[0], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[0],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm0 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[0], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr0 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[0], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa0 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[0], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m0;
        p0m0.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r0;
        p0r0.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a0;
        p0a0.setIdentity();

    tracker_[0].moveekf = tool::ExtendedKalmanFilter{fm0, hm0, j_fm0, j_hm0, u_qm0, u_rm0, p0m0};
    tracker_[0].rotateekf = tool::ExtendedKalmanFilter{fr0, hr0, j_fr0, j_hr0, u_qr0, u_rr0, p0r0};
    tracker_[0].accelerateekf = tool::ExtendedKalmanFilter{fa0, ha0, j_fa0, j_ha0, u_qa0, u_ra0, p0a0};
    }

/////////////////////////////   1
    {
        auto fm1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            x_new(2) += x(3) * dt_[1];
            x_new(4) += x(5) * dt_[1];
            return x_new;
        };
        auto fr1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            return x_new;
        };
        auto fa1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            x_new(2) += x(3) * dt_[1];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[1], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[1], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[1], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[1], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[1], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[1],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm1 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[1], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr1 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[1], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa1 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[1], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m1;
        p0m1.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r1;
        p0r1.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a1;
        p0a1.setIdentity();

    tracker_[1].moveekf = tool::ExtendedKalmanFilter{fm1, hm1, j_fm1, j_hm1, u_qm1, u_rm1, p0m1};
    tracker_[1].rotateekf = tool::ExtendedKalmanFilter{fr1, hr1, j_fr1, j_hr1, u_qr1, u_rr1, p0r1};
    tracker_[1].accelerateekf = tool::ExtendedKalmanFilter{fa1, ha1, j_fa1, j_ha1, u_qa1, u_ra1, p0a1};
    }

/////////////////////////////   2
    {
        auto fm2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            x_new(2) += x(3) * dt_[2];
            x_new(4) += x(5) * dt_[2];
            return x_new;
        };
        auto fr2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            return x_new;
        };
        auto fa2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            x_new(2) += x(3) * dt_[2];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[2], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[2], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[2], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[2], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[2], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[2],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm2 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[2], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr2 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[2], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa2 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[2], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m2;
        p0m2.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r2;
        p0r2.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a2;
        p0a2.setIdentity();

    tracker_[2].moveekf = tool::ExtendedKalmanFilter{fm2, hm2, j_fm2, j_hm2, u_qm2, u_rm2, p0m2};
    tracker_[2].rotateekf = tool::ExtendedKalmanFilter{fr2, hr2, j_fr2, j_hr2, u_qr2, u_rr2, p0r2};
    tracker_[2].accelerateekf = tool::ExtendedKalmanFilter{fa2, ha2, j_fa2, j_ha2, u_qa2, u_ra2, p0a2};
    }

/////////////////////////////   3
    {
        auto fm3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            x_new(2) += x(3) * dt_[3];
            x_new(4) += x(5) * dt_[3];
            return x_new;
        };
        auto fr3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            return x_new;
        };
        auto fa3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            x_new(2) += x(3) * dt_[3];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[3], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[3], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[3], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[3], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[3], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[3],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm3 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[3], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr3 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[3], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa3 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[3], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m3;
        p0m3.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r3;
        p0r3.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a3;
        p0a3.setIdentity();

    tracker_[3].moveekf = tool::ExtendedKalmanFilter{fm3, hm3, j_fm3, j_hm3, u_qm3, u_rm3, p0m3};
    tracker_[3].rotateekf = tool::ExtendedKalmanFilter{fr3, hr3, j_fr3, j_hr3, u_qr3, u_rr3, p0r3};
    tracker_[3].accelerateekf = tool::ExtendedKalmanFilter{fa3, ha3, j_fa3, j_ha3, u_qa3, u_ra3, p0a3};
    }

/////////////////////////////   4
    {
        auto fm4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            x_new(2) += x(3) * dt_[4];
            x_new(4) += x(5) * dt_[4];
            return x_new;
        };
        auto fr4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            return x_new;
        };
        auto fa4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            x_new(2) += x(3) * dt_[4];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[4], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[4], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[4], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[4], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[4], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[4],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm4 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[4], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr4 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[4], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa4 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[4], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m4;
        p0m4.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r4;
        p0r4.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a4;
        p0a4.setIdentity();

    tracker_[4].moveekf = tool::ExtendedKalmanFilter{fm4, hm4, j_fm4, j_hm4, u_qm4, u_rm4, p0m4};
    tracker_[4].rotateekf = tool::ExtendedKalmanFilter{fr4, hr4, j_fr4, j_hr4, u_qr4, u_rr4, p0r4};
    tracker_[4].accelerateekf = tool::ExtendedKalmanFilter{fa4, ha4, j_fa4, j_ha4, u_qa4, u_ra4, p0a4};
    }

/////////////////////////////   5
    {
        auto fm5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            x_new(2) += x(3) * dt_[5];
            x_new(4) += x(5) * dt_[5];
            return x_new;
        };
        auto fr5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            return x_new;
        };
        auto fa5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            x_new(2) += x(3) * dt_[5];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[5], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[5], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[5], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[5], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[5], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[5],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm5 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[5], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr5 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[5], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa5 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[5], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m5;
        p0m5.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r5;
        p0r5.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a5;
        p0a5.setIdentity();

    tracker_[5].moveekf = tool::ExtendedKalmanFilter{fm5, hm5, j_fm5, j_hm5, u_qm5, u_rm5, p0m5};
    tracker_[5].rotateekf = tool::ExtendedKalmanFilter{fr5, hr5, j_fr5, j_hr5, u_qr5, u_rr5, p0r5};
    tracker_[5].accelerateekf = tool::ExtendedKalmanFilter{fa5, ha5, j_fa5, j_ha5, u_qa5, u_ra5, p0a5};
    }

/////////////////////////////   6
    {
        auto fm6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            x_new(2) += x(3) * dt_[6];
            x_new(4) += x(5) * dt_[6];
            return x_new;
        };
        auto fr6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            return x_new;
        };
        auto fa6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            x_new(2) += x(3) * dt_[6];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[6], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[6], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[6], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[6], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[6], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[6],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm6 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[6], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr6 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[6], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa6 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[6], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m6;
        p0m6.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r6;
        p0r6.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a6;
        p0a6.setIdentity();

    tracker_[6].moveekf = tool::ExtendedKalmanFilter{fm6, hm6, j_fm6, j_hm6, u_qm6, u_rm6, p0m6};
    tracker_[6].rotateekf = tool::ExtendedKalmanFilter{fr6, hr6, j_fr6, j_hr6, u_qr6, u_rr6, p0r6};
    tracker_[6].accelerateekf = tool::ExtendedKalmanFilter{fa6, ha6, j_fa6, j_ha6, u_qa6, u_ra6, p0a6};
    }

/////////////////////////////   7
    {
        auto fm7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            x_new(2) += x(3) * dt_[7];
            x_new(4) += x(5) * dt_[7];
            return x_new;
        };
        auto fr7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            return x_new;
        };
        auto fa7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            x_new(2) += x(3) * dt_[7];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[7], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[7], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[7], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[7], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[7], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[7],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm7 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[7], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr7 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[7], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa7 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[7], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m7;
        p0m7.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r7;
        p0r7.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a7;
        p0a7.setIdentity();

    tracker_[7].moveekf = tool::ExtendedKalmanFilter{fm7, hm7, j_fm7, j_hm7, u_qm7, u_rm7, p0m7};
    tracker_[7].rotateekf = tool::ExtendedKalmanFilter{fr7, hr7, j_fr7, j_hr7, u_qr7, u_rr7, p0r7};
    tracker_[7].accelerateekf = tool::ExtendedKalmanFilter{fa7, ha7, j_fa7, j_ha7, u_qa7, u_ra7, p0a7};
    }

/////////////////////////////   11
    {
        auto fm11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            x_new(2) += x(3) * dt_[11];
            x_new(4) += x(5) * dt_[11];
            return x_new;
        };
        auto fr11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            return x_new;
        };
        auto fa11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            x_new(2) += x(3) * dt_[11];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[11], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[11], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[11], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[11], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[11], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[11],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm11 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[11], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr11 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[11], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa11 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[11], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m11;
        p0m11.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r11;
        p0r11.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a11;
        p0a11.setIdentity();

    tracker_[11].moveekf = tool::ExtendedKalmanFilter{fm11, hm11, j_fm11, j_hm11, u_qm11, u_rm11, p0m11};
    tracker_[11].rotateekf = tool::ExtendedKalmanFilter{fr11, hr11, j_fr11, j_hr11, u_qr11, u_rr11, p0r11};
    tracker_[11].accelerateekf = tool::ExtendedKalmanFilter{fa11, ha11, j_fa11, j_ha11, u_qa11, u_ra11, p0a11};
    }

/////////////////////////////   12
    {
        auto fm12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            x_new(2) += x(3) * dt_[12];
            x_new(4) += x(5) * dt_[12];
            return x_new;
        };
        auto fr12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            return x_new;
        };
        auto fa12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            x_new(2) += x(3) * dt_[12];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[12], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[12], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[12], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[12], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[12], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[12],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm12 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[12], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr12 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[12], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa12 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[12], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m12;
        p0m12.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r12;
        p0r12.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a12;
        p0a12.setIdentity();

    tracker_[12].moveekf = tool::ExtendedKalmanFilter{fm12, hm12, j_fm12, j_hm12, u_qm12, u_rm12, p0m12};
    tracker_[12].rotateekf = tool::ExtendedKalmanFilter{fr12, hr12, j_fr12, j_hr12, u_qr12, u_rr12, p0r12};
    tracker_[12].accelerateekf = tool::ExtendedKalmanFilter{fa12, ha12, j_fa12, j_ha12, u_qa12, u_ra12, p0a12};
    }

/////////////////////////////   13
    {
        auto fm13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            x_new(2) += x(3) * dt_[13];
            x_new(4) += x(5) * dt_[13];
            return x_new;
        };
        auto fr13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            return x_new;
        };
        auto fa13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            x_new(2) += x(3) * dt_[13];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[13], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[13], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[13], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[13], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[13], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[13],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm13 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[13], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr13 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[13], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa13 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[13], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m13;
        p0m13.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r13;
        p0r13.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a13;
        p0a13.setIdentity();

    tracker_[13].moveekf = tool::ExtendedKalmanFilter{fm13, hm13, j_fm13, j_hm13, u_qm13, u_rm13, p0m13};
    tracker_[13].rotateekf = tool::ExtendedKalmanFilter{fr13, hr13, j_fr13, j_hr13, u_qr13, u_rr13, p0r13};
    tracker_[13].accelerateekf = tool::ExtendedKalmanFilter{fa13, ha13, j_fa13, j_ha13, u_qa13, u_ra13, p0a13};
    }

    }

    int Controler_l::getAimingPoint(std::vector<base::Armor> armors, cv::Point3f &aiming_point, double timestamp, int8_t *provirty_list, int *attack_id, cv::Point3f &gun_point)
    {   
        bool time_seted[14] = {false};
        double time[14];
        for (int i = 0; i < 14; i++) {
            time[i] = timestamp;
        }

        bool id_seted[14] = {false};
        std::vector<base::Armor> kind_armors[14];
        std::vector<int> armors_ids;
        for (auto &armor : armors) {
            kind_armors[armor.num_id].push_back(armor);
            if (!id_seted[armor.num_id]) {
                armors_ids.push_back(armor.num_id);
                id_seted[armor.num_id] = true;
            }
        }

        bool is_tracking[14] = {false};

        for (int id = 0; id < 14; id++) {
            if (id == 8 || id == 9 || id==10)
                continue;

            if (tracker_[id].tracker_state == base::LOST) {
                is_tracking[id] = false;
                tracker_[id].init(kind_armors[id]);
                last_time_[id] = time[id];
            } else {
                dt_[id] = time[id] - last_time_[id];
                // std::cout << "dt =    " << id << "    " << dt_[id] << '\n';

                if (tracker_[7].tracked_id == 7) {
                    tracker_[7].lost_thres = static_cast<int>(lost_time_thres_ / dt_[7]) * 5;
                } else if (tracker_[11].tracked_id == 11 || tracker_[12].tracked_id == 12 || tracker_[13].tracked_id == 13) {
                    tracker_[11].lost_thres = static_cast<int>(lost_time_thres_ / dt_[11]) * 2;
                    tracker_[12].lost_thres = static_cast<int>(lost_time_thres_ / dt_[12]) * 2;
                    tracker_[13].lost_thres = static_cast<int>(lost_time_thres_ / dt_[13]) * 2;
                }

                tracker_[id].update(kind_armors[id]);

                if (tracker_[id].tracker_state == base::DETECTING) {
                    is_tracking[id] = false;
                } else if ( tracker_[id].tracker_state == base::TRACKING ||
                            tracker_[id].tracker_state == base::TEMP_LOST) {
                    is_tracking[id] = true;
                }

                last_time_[id] = time[id];
            }
        }

        for (int i = 0; i < 14; i++)
        {
            int8_t aim_id = provirty_list[i];   
    
            if (is_tracking[aim_id])
            {
                *attack_id = aim_id;

                // get car state
                const auto & move_state = tracker_[aim_id].move_target_state;
                const auto & rotate_state = tracker_[aim_id].rotate_target_state;
                const auto & accelerate_state = tracker_[aim_id].accelerate_target_state;
                double yaw = rotate_state[0], r1 = move_state[7], r2 = tracker_[aim_id].another_r;
                double xc = move_state[0], yc = move_state[2], za = move_state[4];
                double vx = move_state[1], vy = move_state[3], vz = move_state[5];
                double dz = tracker_[aim_id].dz; 
                double v_yaw = rotate_state[1];
                double ax = accelerate_state[1], ay = accelerate_state[3];
                // std::cout << "r = " << move_state[7] << '\n';

                if (vx > 10)
                    vx = 0;
                if (vy > 10)
                    vy = 0;
                if (vz > 10)
                    vz = 0;

                // predict
                cv::Point3d p_center = cv::Point3d(xc, yc, za);
                cv::Point3d velocity_linear = cv::Point3d(vx, vy, vz);
                cv::Point3d acceleration = cv::Point3d(ax, ay, 0);

                if (aim_id == 7)
                    delay_ = 0.144290;
                if (v_yaw < 0)
                    delay_ = 0.149290;
                double all_time = ballistic_solver_.getAngleTimel(p_center * 1000).z + delay_;

                cv::Point3d linear_change = cv::Point3d(velocity_linear.x * (1.0 * all_time - 0.07) + 1 / 2 * acceleration.x * pow(all_time, 2),
                                                        velocity_linear.y * (1.0 * all_time - 0.07) + 1 / 2 * acceleration.y * pow(all_time, 2),
                                                        velocity_linear.z * (0.1 * all_time - 0.07) + 1 / 2 * acceleration.z * pow(all_time, 2));
                cv::Point3d gun_l_change = cv::Point3d( velocity_linear.x * (1.0 * all_time + 0.02) + 1 / 2 * acceleration.x * pow(all_time, 2),
                                                        velocity_linear.y * (1.0 * all_time + 0.02) + 1 / 2 * acceleration.y * pow(all_time, 2),
                                                        velocity_linear.z * (0.1 * all_time + 0.00) + 1 / 2 * acceleration.z * pow(all_time, 2));

                cv::Point3d p_predict_center = p_center + linear_change;  // 预测中心点
                cv::Point3d gun_predict_center = p_center + gun_l_change; // 预测中心点

                double yaw_predict = yaw + v_yaw * all_time;              // 预测yaw
                double gun_yaw_predct = yaw + v_yaw * all_time;           // 预测yaw

                // get armors num
                int armors_num = 4;
                if (aim_id == 11 || aim_id == 12 || aim_id == 13) {
                    armors_num = 2;
                } else if (aim_id == 7) {
                    armors_num = 3;
                }

                cv::Point3d p[armors_num];
                cv::Point3d gunp[armors_num];

                cv::Point2d p_predict_center_2d(p_predict_center.x, p_predict_center.y);
                // cv::Point2d gun_predict_center_2d(gun_predict_center.x, gun_predict_center.y);
                double angle_to_center[armors_num];

                bool is_current_pair = true;
                double r = 0;
                int min_dis_point_index = 0;
                double min_dis = DBL_MAX;
                // 整车模型建立
                for (int i = 0; i < armors_num; i++) {
                    double tmp_yaw = yaw_predict + i * (2.0 * M_PI / armors_num);
                    double gun_temp_yaw = gun_yaw_predct + i * (2.0 * M_PI / armors_num);
                    if (armors_num == 4) {
                        r = is_current_pair ? r1 : r2;
                        gunp[i].z = za + (is_current_pair ? 0 : dz);
                        p[i].z = za + (is_current_pair ? 0 : dz);
                        is_current_pair = !is_current_pair;
                    } else {
                        r = r1;
                        p[i].z = za;
                        gunp[i].z = za;
                    }

                    p[i].x = (p_predict_center.x - r * cos(tmp_yaw));
                    gunp[i].x = (gun_predict_center.x - r * cos(gun_temp_yaw));
                    p[i].y = (p_predict_center.y - r * sin(tmp_yaw));
                    gunp[i].y = (gun_predict_center.y - r * sin(gun_temp_yaw));

                    double dis = sqrt(pow(p[i].x, 2) + pow(p[i].y, 2) + pow(p[i].z, 2));
                    if (dis < min_dis) {
                        if (abs(dis - min_dis) > 0) {
                            min_dis = dis;
                            min_dis_point_index = i;        
                        }
                    }

                    cv::Point2d p_2d(p[i].x - p_predict_center_2d.x, p[i].y - p_predict_center_2d.y);
                    angle_to_center[i] = (p_2d.x * p_predict_center_2d.x +
                                          p_2d.y * p_predict_center_2d.y) /
                                         (sqrt(p_2d.x * p_2d.x + p_2d.y * p_2d.y) *
                                          sqrt(p_predict_center_2d.x * p_predict_center_2d.x +
                                               p_predict_center_2d.y * p_predict_center_2d.y));  // 圆心和原点连线，圆心与装甲板连线，两者夹角的余弦值
                }

                // 求解最优点
                double limit_area = 0.30 + abs(v_yaw) * 0.05;
                if (limit_area > 0.99)
                    limit_area = 0.99;

                // 前哨站
                if (aim_id == 7)
                {
                    limit_area = 0.53;

                    // if ( (v_yaw > 0 && p_predict_center.x > 0) || 
                    //      (v_yaw < 0 && p_predict_center.x < 0) )
                    // {   
                        if ( (angle_to_center[min_dis_point_index] < -limit_area
                             && p[min_dis_point_index].y > p_predict_center.y)
                             ||
                             (angle_to_center[min_dis_point_index] < -(limit_area + 0.28)
                             && p[min_dis_point_index].y < p_predict_center.y)
                        ) {
                            aiming_point = p[min_dis_point_index];
                            // gun_point = p_predict_center;
                            gun_point = gunp[min_dis_point_index];

                            return 1;
                        } else {
                            int next_index_1 = (min_dis_point_index + 1) % armors_num;
                            int next_index_2 = min_dis_point_index - 1;
                            if (min_dis_point_index == 0)
                                next_index_2 = armors_num - 1;
                            cv::Point3f next_point_1 = p[next_index_1];
                            cv::Point3f next_point_2 = p[next_index_2];
                            cv::Point3f next_point = next_point_1;
                            cv::Point3f gun_next_point_1 = gunp[next_index_1];
                            cv::Point3f gun_next_point_2 = gunp[next_index_2];
                            cv::Point3f gun_next_point = next_point_1;
                            if (v_yaw < 0) {
                                next_point = next_point_1;
                                gun_next_point = gun_next_point_1;
                            } else {
                                next_point = next_point_2;
                                gun_next_point = gun_next_point_2;
                            }

                            aiming_point = next_point;
                            gun_point = p_predict_center;
                            // gun_point = gun_next_point;

                            if (aiming_point.x * aiming_point.x + aiming_point.y * aiming_point.y
                               > p_predict_center.x * p_predict_center.x + p_predict_center.y * p_predict_center.y)
                                return 2;
                            else
                                return 1;
                        }
                    // }
                }

                if (angle_to_center[min_dis_point_index] < -limit_area && tracker_[aim_id].lost_count_ <= 5) 
                {
                    aiming_point = p[min_dis_point_index];
                    gun_point = gunp[min_dis_point_index];
                    if (abs(v_yaw) > 2.6)
                        gun_point = p_predict_center;

                    return 1;
                }

                else
                {
                    int next_index_1 = (min_dis_point_index + 1) % armors_num;
                    int next_index_2 = min_dis_point_index - 1;
                    if (min_dis_point_index == 0)
                        next_index_2 = armors_num - 1;
                    cv::Point3f next_point_1 = p[next_index_1];
                    cv::Point3f next_point_2 = p[next_index_2];
                    cv::Point3f next_point = next_point_1;
                    cv::Point3f gun_next_point_1 = gunp[next_index_1];
                    cv::Point3f gun_next_point_2 = gunp[next_index_2];
                    cv::Point3f gun_next_point = next_point_1;
                    if (v_yaw < 0) {
                        next_point = next_point_1;
                        gun_next_point = gun_next_point_1;
                    } else {
                        next_point = next_point_2;
                        gun_next_point = gun_next_point_2;
                    }

                    aiming_point = next_point;
                    gun_point = gun_next_point;
                    if (abs(v_yaw) > 2.6)
                        gun_point = p_predict_center;
                    return 2;
                }
            }
            else if (provirty_list[i+1] != -1)
                continue;
            else {
                *attack_id = -1;
                aiming_point = cv::Point3f(0,0,0);
                gun_point = cv::Point3f(0,0,0);

                return 3;
            }
        }
    }

    bool Controler_l::judgeFire(cv::Point3f aiming_point_camera, double v_yaw, cv::Point2f &aim_point_2d_out, bool print, int id)
    {
        // if (id == 7 && v_yaw == 0)
        //     return false;

        cv::Point2f aim_point_2d;

        cv::Mat a;
        a = (cv::Mat_<double>(3, 1) << aiming_point_camera.x, aiming_point_camera.y, aiming_point_camera.z);
        cv::Mat b = camera_matrix_ * a;
        cv::Point3d b_new(b);
        b_new = b_new / b_new.z;
        aim_point_2d.x = std::ceil(b_new.x);
        aim_point_2d.y = std::ceil(b_new.y);
        aim_point_2d_out = aim_point_2d;

        double x = aim_point_2d.x;
        double delta_x = abs(x - this->true_x_);
        // if (print)
        //     std::cout << "x  is  " << x << std::endl;

        double fire_area = abs(80 / v_yaw);
        if (abs(v_yaw) == 2.512)
        {
            // std::cout << "ok" << '\n';
            fire_area = camera_matrix_.at<double>(0, 0) * 55 / aiming_point_camera.z;}
        else
        { 
            // std::cout << "sb" << '\n';
            fire_area = camera_matrix_.at<double>(0, 0) * 80 / aiming_point_camera.z;
        }

        if (id == 1 || id == 11 || id == 12 || id == 13)
            fire_area = camera_matrix_.at<double>(0, 0) * 140 / aiming_point_camera.z;

        // std::cout << "v_yaw is " << v_yaw << std::endl;                  
        // std::cout << "fire_area is " << fire_area << std::endl;
                
        if (fire_area < 5) {
            fire_area = 5;
        }
        if (fire_area > 150) {
            fire_area = 150;
        }

        if (delta_x < fire_area) {
            return true;
        } else {
            return false;
        }
    }

    bool Controler_l::getParam(cv::Mat camera_matrix)
    {
        this->camera_matrix_ = camera_matrix;
        if (camera_matrix.empty()) {
            std::cout << "camera matrix while get param empty" << std::endl;
            return false;
        } else {
            return true;
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////

    Controler_r::Controler_r()
    {
        cv::FileStorage fs("./src/Algorithm/configure/Processer/controler/param_r.xml", cv::FileStorage::READ);

        if (!fs.isOpened()) {
            std::cout << "open processer controler param fail" << std::endl;
            exit(0);
        }

        fs["lost_time_thres"] >> lost_time_thres_;
        fs["s2q_xyz"] >> s2q_xyz_;
        fs["s2q_myaw"] >> s2q_myaw_;
        fs["s2q_ryaw"] >> s2q_ryaw_;
        fs["s2q_a_xy"] >> s2q_a_xy_;

        fs["s2q_r"] >> s2q_r_;

        fs["r_xyz_factor"] >> r_xyz_factor;
        fs["r_myaw"] >> r_myaw;
        fs["r_ryaw"] >> r_ryaw;
        fs["r_a_xy"] >> r_a_xy;
        
        fs["delay"] >> delay_;
        fs["odelay"] >> o_delay;
        fs["true_x"] >> true_x_;

        fs["gun_pitch_offset"] >> this->gun_pitch_offset_;
        fs["gun_yaw_offset"] >> this->gun_yaw_offset_;

        fs.release();

        // EKF
        // xa = x_armor, xc = x_robot_center
        // state: xc, v_xc, yc, v_yc, za, v_za, r
        // measurement: xa, ya, za, yaw

        // move ekf
        // state: xc, v_xc, yc, v_yc, za, v_za, yaw, r 
        // measurement: xa, ya, za, yaw 

        // rotate ekf
        // state: yaw, vyaw 
        // measurement: yaw

        // accelerate ekf
        // state: v_xc, a_xc, v_yc, a_yc
        // measurement: v_xc, v_yc

/////////////////////////////   0
    {
        auto fm0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            x_new(2) += x(3) * dt_[0];
            x_new(4) += x(5) * dt_[0];
            return x_new;
        };
        auto fr0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            return x_new;
        };
        auto fa0 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[0];
            x_new(2) += x(3) * dt_[0];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[0], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[0], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[0], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[0], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa0 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[0], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[0],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha0 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha0 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm0 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[0], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr0 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[0], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa0 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[0], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra0 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m0;
        p0m0.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r0;
        p0r0.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a0;
        p0a0.setIdentity();

    tracker_[0].moveekf = tool::ExtendedKalmanFilter{fm0, hm0, j_fm0, j_hm0, u_qm0, u_rm0, p0m0};
    tracker_[0].rotateekf = tool::ExtendedKalmanFilter{fr0, hr0, j_fr0, j_hr0, u_qr0, u_rr0, p0r0};
    tracker_[0].accelerateekf = tool::ExtendedKalmanFilter{fa0, ha0, j_fa0, j_ha0, u_qa0, u_ra0, p0a0};
    }

/////////////////////////////   1
    {
        auto fm1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            x_new(2) += x(3) * dt_[1];
            x_new(4) += x(5) * dt_[1];
            return x_new;
        };
        auto fr1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            return x_new;
        };
        auto fa1 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[1];
            x_new(2) += x(3) * dt_[1];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[1], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[1], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[1], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[1], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa1 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[1], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[1],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha1 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha1 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm1 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[1], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr1 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[1], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa1 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[1], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra1 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m1;
        p0m1.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r1;
        p0r1.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a1;
        p0a1.setIdentity();

    tracker_[1].moveekf = tool::ExtendedKalmanFilter{fm1, hm1, j_fm1, j_hm1, u_qm1, u_rm1, p0m1};
    tracker_[1].rotateekf = tool::ExtendedKalmanFilter{fr1, hr1, j_fr1, j_hr1, u_qr1, u_rr1, p0r1};
    tracker_[1].accelerateekf = tool::ExtendedKalmanFilter{fa1, ha1, j_fa1, j_ha1, u_qa1, u_ra1, p0a1};
    }

/////////////////////////////   2
    {
        auto fm2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            x_new(2) += x(3) * dt_[2];
            x_new(4) += x(5) * dt_[2];
            return x_new;
        };
        auto fr2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            return x_new;
        };
        auto fa2 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[2];
            x_new(2) += x(3) * dt_[2];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[2], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[2], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[2], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[2], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa2 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[2], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[2],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha2 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha2 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm2 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[2], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr2 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[2], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa2 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[2], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra2 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m2;
        p0m2.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r2;
        p0r2.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a2;
        p0a2.setIdentity();

    tracker_[2].moveekf = tool::ExtendedKalmanFilter{fm2, hm2, j_fm2, j_hm2, u_qm2, u_rm2, p0m2};
    tracker_[2].rotateekf = tool::ExtendedKalmanFilter{fr2, hr2, j_fr2, j_hr2, u_qr2, u_rr2, p0r2};
    tracker_[2].accelerateekf = tool::ExtendedKalmanFilter{fa2, ha2, j_fa2, j_ha2, u_qa2, u_ra2, p0a2};
    }

/////////////////////////////   3
    {
        auto fm3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            x_new(2) += x(3) * dt_[3];
            x_new(4) += x(5) * dt_[3];
            return x_new;
        };
        auto fr3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            return x_new;
        };
        auto fa3 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[3];
            x_new(2) += x(3) * dt_[3];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[3], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[3], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[3], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[3], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa3 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[3], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[3],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha3 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha3 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm3 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[3], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr3 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[3], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa3 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[3], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra3 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m3;
        p0m3.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r3;
        p0r3.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a3;
        p0a3.setIdentity();

    tracker_[3].moveekf = tool::ExtendedKalmanFilter{fm3, hm3, j_fm3, j_hm3, u_qm3, u_rm3, p0m3};
    tracker_[3].rotateekf = tool::ExtendedKalmanFilter{fr3, hr3, j_fr3, j_hr3, u_qr3, u_rr3, p0r3};
    tracker_[3].accelerateekf = tool::ExtendedKalmanFilter{fa3, ha3, j_fa3, j_ha3, u_qa3, u_ra3, p0a3};
    }

/////////////////////////////   4
    {
        auto fm4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            x_new(2) += x(3) * dt_[4];
            x_new(4) += x(5) * dt_[4];
            return x_new;
        };
        auto fr4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            return x_new;
        };
        auto fa4 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[4];
            x_new(2) += x(3) * dt_[4];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[4], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[4], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[4], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[4], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa4 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[4], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[4],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha4 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha4 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm4 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[4], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr4 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[4], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa4 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[4], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra4 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m4;
        p0m4.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r4;
        p0r4.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a4;
        p0a4.setIdentity();

    tracker_[4].moveekf = tool::ExtendedKalmanFilter{fm4, hm4, j_fm4, j_hm4, u_qm4, u_rm4, p0m4};
    tracker_[4].rotateekf = tool::ExtendedKalmanFilter{fr4, hr4, j_fr4, j_hr4, u_qr4, u_rr4, p0r4};
    tracker_[4].accelerateekf = tool::ExtendedKalmanFilter{fa4, ha4, j_fa4, j_ha4, u_qa4, u_ra4, p0a4};
    }

/////////////////////////////   5
    {
        auto fm5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            x_new(2) += x(3) * dt_[5];
            x_new(4) += x(5) * dt_[5];
            return x_new;
        };
        auto fr5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            return x_new;
        };
        auto fa5 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[5];
            x_new(2) += x(3) * dt_[5];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[5], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[5], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[5], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[5], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa5 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[5], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[5],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha5 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha5 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm5 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[5], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr5 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[5], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa5 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[5], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra5 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m5;
        p0m5.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r5;
        p0r5.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a5;
        p0a5.setIdentity();

    tracker_[5].moveekf = tool::ExtendedKalmanFilter{fm5, hm5, j_fm5, j_hm5, u_qm5, u_rm5, p0m5};
    tracker_[5].rotateekf = tool::ExtendedKalmanFilter{fr5, hr5, j_fr5, j_hr5, u_qr5, u_rr5, p0r5};
    tracker_[5].accelerateekf = tool::ExtendedKalmanFilter{fa5, ha5, j_fa5, j_ha5, u_qa5, u_ra5, p0a5};
    }

/////////////////////////////   6
    {
        auto fm6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            x_new(2) += x(3) * dt_[6];
            x_new(4) += x(5) * dt_[6];
            return x_new;
        };
        auto fr6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            return x_new;
        };
        auto fa6 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[6];
            x_new(2) += x(3) * dt_[6];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[6], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[6], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[6], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[6], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa6 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[6], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[6],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha6 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha6 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm6 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[6], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr6 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[6], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa6 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[6], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra6 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m6;
        p0m6.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r6;
        p0r6.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a6;
        p0a6.setIdentity();

    tracker_[6].moveekf = tool::ExtendedKalmanFilter{fm6, hm6, j_fm6, j_hm6, u_qm6, u_rm6, p0m6};
    tracker_[6].rotateekf = tool::ExtendedKalmanFilter{fr6, hr6, j_fr6, j_hr6, u_qr6, u_rr6, p0r6};
    tracker_[6].accelerateekf = tool::ExtendedKalmanFilter{fa6, ha6, j_fa6, j_ha6, u_qa6, u_ra6, p0a6};
    }

/////////////////////////////   7
    {
        auto fm7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            x_new(2) += x(3) * dt_[7];
            x_new(4) += x(5) * dt_[7];
            return x_new;
        };
        auto fr7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            return x_new;
        };
        auto fa7 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[7];
            x_new(2) += x(3) * dt_[7];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[7], 0,   0,      0,   0,      0,  0,
                    0,   1,      0,   0,      0,   0,      0,  0,
                    0,   0,      1,   dt_[7], 0,   0,      0,  0,
                    0,   0,      0,   1,      0,   0,      0,  0,
                    0,   0,      0,   0,      1,   dt_[7], 0,  0,
                    0,   0,      0,   0,      0,   1,      0,  0,
                    0,   0,      0,   0,      0,   0,      1,  0,
                    0,   0,      0,   0,      0,   0,      0,  1;
            // clang-format on
            return f;
        };
        auto j_fr7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[7], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa7 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[7], 0,   0,
                    0,   1,      0,   0,
                    0,   0,      1,   dt_[7],
                    0,   0,      0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha7 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha7 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm7 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[7], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr7 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[7], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa7 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[7], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra7 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m7;
        p0m7.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r7;
        p0r7.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a7;
        p0a7.setIdentity();

    tracker_[7].moveekf = tool::ExtendedKalmanFilter{fm7, hm7, j_fm7, j_hm7, u_qm7, u_rm7, p0m7};
    tracker_[7].rotateekf = tool::ExtendedKalmanFilter{fr7, hr7, j_fr7, j_hr7, u_qr7, u_rr7, p0r7};
    tracker_[7].accelerateekf = tool::ExtendedKalmanFilter{fa7, ha7, j_fa7, j_ha7, u_qa7, u_ra7, p0a7};
    }

/////////////////////////////   11
    {
        auto fm11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            x_new(2) += x(3) * dt_[11];
            x_new(4) += x(5) * dt_[11];
            return x_new;
        };
        auto fr11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            return x_new;
        };
        auto fa11 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[11];
            x_new(2) += x(3) * dt_[11];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[11], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[11], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[11], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[11], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa11 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[11], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[11],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha11 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha11 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm11 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[11], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr11 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[11], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa11 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[11], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra11 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m11;
        p0m11.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r11;
        p0r11.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a11;
        p0a11.setIdentity();

    tracker_[11].moveekf = tool::ExtendedKalmanFilter{fm11, hm11, j_fm11, j_hm11, u_qm11, u_rm11, p0m11};
    tracker_[11].rotateekf = tool::ExtendedKalmanFilter{fr11, hr11, j_fr11, j_hr11, u_qr11, u_rr11, p0r11};
    tracker_[11].accelerateekf = tool::ExtendedKalmanFilter{fa11, ha11, j_fa11, j_ha11, u_qa11, u_ra11, p0a11};
    }

/////////////////////////////   12
    {
        auto fm12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            x_new(2) += x(3) * dt_[12];
            x_new(4) += x(5) * dt_[12];
            return x_new;
        };
        auto fr12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            return x_new;
        };
        auto fa12 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[12];
            x_new(2) += x(3) * dt_[12];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[12], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[12], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[12], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[12], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa12 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[12], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[12],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha12 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha12 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm12 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[12], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr12 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[12], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa12 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[12], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra12 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m12;
        p0m12.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r12;
        p0r12.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a12;
        p0a12.setIdentity();

    tracker_[12].moveekf = tool::ExtendedKalmanFilter{fm12, hm12, j_fm12, j_hm12, u_qm12, u_rm12, p0m12};
    tracker_[12].rotateekf = tool::ExtendedKalmanFilter{fr12, hr12, j_fr12, j_hr12, u_qr12, u_rr12, p0r12};
    tracker_[12].accelerateekf = tool::ExtendedKalmanFilter{fa12, ha12, j_fa12, j_ha12, u_qa12, u_ra12, p0a12};
    }

/////////////////////////////   13
    {
        auto fm13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            x_new(2) += x(3) * dt_[13];
            x_new(4) += x(5) * dt_[13];
            return x_new;
        };
        auto fr13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            return x_new;
        };
        auto fa13 = [this](const Eigen::VectorXd & x) {
            Eigen::VectorXd x_new = x;
            x_new(0) += x(1) * dt_[13];
            x_new(2) += x(3) * dt_[13];
            return x_new;
        };
        // J_f - Jacobian of process function
        auto j_fm13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(8, 8);
            // clang-format off
            f <<    1,   dt_[13], 0,   0,       0,   0,       0,  0,
                    0,   1,       0,   0,       0,   0,       0,  0,
                    0,   0,       1,   dt_[13], 0,   0,       0,  0,
                    0,   0,       0,   1,       0,   0,       0,  0,
                    0,   0,       0,   0,       1,   dt_[13], 0,  0,
                    0,   0,       0,   0,       0,   1,       0,  0,
                    0,   0,       0,   0,       0,   0,       1,  0,
                    0,   0,       0,   0,       0,   0,       0,  1;
            // clang-format on
            return f;
        };
        auto j_fr13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(2, 2);
            // clang-format off
            f <<    1,   dt_[13], 
                    0,   1;
            // clang-format on
            return f;
        };
        auto j_fa13 = [this](const Eigen::VectorXd &) {
            Eigen::MatrixXd f(4, 4);
            // clang-format off
            f <<    1,   dt_[13], 0,   0,
                    0,   1,       0,   0,
                    0,   0,       1,   dt_[13],
                    0,   0,       0,   1;
            // clang-format on
            return f;
        };
        // h - Observation function
        auto hm13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(4);
            double xc = x(0), yc = x(2), yaw = x(6), r = x(7);
            z(0) = xc - r * cos(yaw);  // xa
            z(1) = yc - r * sin(yaw);  // ya
            z(2) = x(4);               // za
            z(3) = x(6);               // yaw
            return z;
        };
        auto hr13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(1);
            z(0) = x(0);               // yaw
            return z;
        };
        auto ha13 = [](const Eigen::VectorXd & x) {
            Eigen::VectorXd z(2);
            z(0) = x(0);               // v_xc
            z(1) = x(2);               // v_yc
            return z;
        };
        // J_h - Jacobian of observation function
        auto j_hm13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(4, 8);
            double yaw = x(6), r = x(7);
            // clang-format off
            //    xc    v_xc  yc    v_yc  za    v_za  yaw            r
            h <<  1,    0,    0,    0,    0,    0,    r*sin(yaw),    -cos(yaw),
                  0,    0,    1,    0,    0,    0,    -r*cos(yaw),   -sin(yaw),
                  0,    0,    0,    0,    1,    0,    0,             0,
                  0,    0,    0,    0,    0,    0,    1,             0;
            // clang-format on
            return h;
        };
        auto j_hr13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(1, 2);
            // clang-format off
            //     yaw    v_yaw
            h <<   1,     0;
            // clang-format on
            return h;
        };
        auto j_ha13 = [](const Eigen::VectorXd & x) {
            Eigen::MatrixXd h(2, 4);
            // clang-format off
            //     v_xc    a_xc   v_yc   a_yc
            h <<   1,      0,     0,     0,
                   0,      0,     1,     0;
            // clang-format on
            return h;
        };
        // update_Q - process noise covariance matrix
        auto u_qm13 = [this]() {
            Eigen::MatrixXd q(8, 8);
            double t = dt_[13], x = s2q_xyz_, y = s2q_myaw_, r = s2q_r_;
            double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx =  pow(t, 2) * x;
            double q_y_y = 0.1 * pow(t, 4) / 4 * y;
            double q_r = pow(t, 4) / 4 * r;
            // clang-format off
            //    xc        v_xc      yc        v_yc      za        v_za      yaw       r
            q <<  q_x_x,    q_x_vx,   0,        0,        0,        0,        0,        0,
                  q_x_vx,   q_vx_vx,  0,        0,        0,        0,        0,        0,
                  0,        0,        q_x_x,    q_x_vx,   0,        0,        0,        0,
                  0,        0,        q_x_vx,   q_vx_vx,  0,        0,        0,        0,
                  0,        0,        0,        0,        q_x_x,    q_x_vx,   0,        0,
                  0,        0,        0,        0,        q_x_vx,   q_vx_vx,  0,        0,
                  0,        0,        0,        0,        0,        0,        q_y_y,    0,
                  0,        0,        0,        0,        0,        0,        0,        q_r;
            // clang-format on
            return q;
        };
        auto u_qr13 = [this]() {
            Eigen::MatrixXd q(2, 2);
            double t = dt_[13], y = s2q_ryaw_;
            double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
            // clang-format off
            //    yaw       v_yaw
            q <<  q_y_y,    q_y_vy,
                  q_y_vy,   q_vy_vy;
            // clang-format on
            return q;
        };
        auto u_qa13 = [this]() {
            Eigen::MatrixXd q(4, 4);
            double t = dt_[13], y = s2q_a_xy_;
            double q_vx_vx = pow(t, 4) / 4 * y, q_vx_ax = 0.1 * pow(t, 3) / 2 * y, q_ax_ax = pow(t, 2) * y;
            // clang-format off
            //    v_xc      a_xc      v_yc      a_yc
            q <<  q_vx_vx,  q_vx_ax,  0,        0,
                  q_vx_ax,  q_ax_ax,  0,        0,
                  0,        0,        q_vx_vx,  q_vx_ax,
                  0,        0,        q_vx_ax,  q_ax_ax;
            // clang-format on
            return q;
        };
        // update_R - measurement noise covariance matrix
        auto u_rm13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 4> r;
            double x = r_xyz_factor;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_myaw;
            return r;
        };
        auto u_rr13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 1> r;
            r.diagonal() << r_ryaw;
            return r;
        };
        auto u_ra13 = [this](const Eigen::VectorXd & z) {
            Eigen::DiagonalMatrix<double, 2> r;
            double x = r_a_xy;
            r.diagonal() << abs(x * z[0]), abs(x * z[1]);
            return r;
        };
        // P - error estimate covariance matrix
        Eigen::DiagonalMatrix<double, 8> p0m13;
        p0m13.setIdentity();
        Eigen::DiagonalMatrix<double, 2> p0r13;
        p0r13.setIdentity();
        Eigen::DiagonalMatrix<double, 4> p0a13;
        p0a13.setIdentity();

    tracker_[13].moveekf = tool::ExtendedKalmanFilter{fm13, hm13, j_fm13, j_hm13, u_qm13, u_rm13, p0m13};
    tracker_[13].rotateekf = tool::ExtendedKalmanFilter{fr13, hr13, j_fr13, j_hr13, u_qr13, u_rr13, p0r13};
    tracker_[13].accelerateekf = tool::ExtendedKalmanFilter{fa13, ha13, j_fa13, j_ha13, u_qa13, u_ra13, p0a13};
    }

    }
    
    int Controler_r::getAimingPoint(std::vector<base::Armor> armors, cv::Point3f &aiming_point, double timestamp, int8_t *provirty_list, int *attack_id, cv::Point3f &gun_point, cv::Point3f &aim2_point, bool center)
    {
        bool time_seted[14] = {false};
        double time[14];
        for (int i = 0; i < 14; i++) {
            time[i] = timestamp;
        }

        bool id_seted[14] = {false};
        std::vector<base::Armor> kind_armors[14];
        std::vector<int> armors_ids;
        for (auto &armor : armors) {
            kind_armors[armor.num_id].push_back(armor);
            if (!id_seted[armor.num_id]) {
                armors_ids.push_back(armor.num_id);
                id_seted[armor.num_id] = true;
            }
        }

        bool is_tracking[14] = {false};

        for (int id = 0; id < 14; id++)
        {
            if (tracker_[id].tracker_state == base::LOST)
            {
                is_tracking[id] = false;
                tracker_[id].init(kind_armors[id]);
                last_time_[id] = time[id];
            }
            else
            {
                dt_[id] = time[id] - last_time_[id];

                if (tracker_[7].tracked_id == 7 ) {
                    tracker_[7].lost_thres = static_cast<int>(lost_time_thres_ / dt_[7]) * 5;
                }
                else if (tracker_[11].tracked_id == 11 || tracker_[12].tracked_id == 12 || tracker_[13].tracked_id == 13) {
                    tracker_[11].lost_thres = static_cast<int>(lost_time_thres_ / dt_[11]) * 2;
                    tracker_[12].lost_thres = static_cast<int>(lost_time_thres_ / dt_[12]) * 2;
                    tracker_[13].lost_thres = static_cast<int>(lost_time_thres_ / dt_[13]) * 2;
                }

                tracker_[id].update(kind_armors[id]);

                if (tracker_[id].tracker_state == base::DETECTING) {
                    is_tracking[id] = false;
                } else if (
                        tracker_[id].tracker_state == base::TRACKING ||
                        tracker_[id].tracker_state == base::TEMP_LOST) {
                    is_tracking[id] = true;
                }

                last_time_[id] = time[id];
            }
        }

        for (int i = 0; i < 14; i++)
        {
            int8_t aim_id = provirty_list[i];

            if (is_tracking[aim_id])
            {
                *attack_id = aim_id;

                // get car state
                const auto & move_state = tracker_[aim_id].move_target_state;
                const auto & rotate_state = tracker_[aim_id].rotate_target_state;
                const auto & accelerate_state = tracker_[aim_id].accelerate_target_state;
                double yaw = rotate_state[0], r1 = move_state[7], r2 = tracker_[aim_id].another_r;
                double xc = move_state[0], yc = move_state[2], za = move_state[4];
                double vx = move_state[1], vy = move_state[3], vz = move_state[5];
                double dz = tracker_[aim_id].dz;
                double v_yaw = rotate_state[1];
                double ax = accelerate_state[1], ay = accelerate_state[3];

                if (vx > 10)
                    vx = 0;
                if (vy > 10)
                    vy = 0;
                if (vz > 10)
                    vz = 0;

                // predict
                cv::Point3d p_center = cv::Point3d(xc, yc, za);
                cv::Point3d velocity_linear = cv::Point3d(vx, vy, vz);
                cv::Point3d acceleration = cv::Point3d(ax, ay, 0);

                if (aim_id == 7)
                    delay_ = o_delay;
                if (aim_id == 7 && v_yaw < 0)
                    delay_ -= 0.012;
                double all_time = ballistic_solver_.getAngleTimer(p_center * 1000).z + delay_;

                cv::Point3d linear_change = cv::Point3d(velocity_linear.x * (1.0 * all_time - 0.07) + 1 / 2 * acceleration.x * pow(all_time, 2),
                                                        velocity_linear.y * (1.0 * all_time - 0.07) + 1 / 2 * acceleration.y * pow(all_time, 2),
                                                        velocity_linear.z * (0.1 * all_time - 0.00) + 1 / 2 * acceleration.z * pow(all_time, 2));
                cv::Point3d gun_r_change = cv::Point3d( velocity_linear.x * (1.0 * all_time + 0.02) + 1 / 2 * acceleration.x * pow(all_time, 2),
                                                        velocity_linear.y * (1.0 * all_time + 0.02) + 1 / 2 * acceleration.y * pow(all_time, 2),
                                                        velocity_linear.z * (0.1 * all_time + 0.00) + 1 / 2 * acceleration.z * pow(all_time, 2));

                // cv::Point3d p_predict_center = p_center + linear_change;  // 预测中心点
                cv::Point3d p_predict_center = p_center;  // 预测中心点
                // cv::Point3d gun_predict_center = p_center + gun_r_change; // 预测中心点
                cv::Point3d gun_predict_center = p_center; // 预测中心点

                double yaw_predict = yaw + v_yaw * all_time;       // 预测yaw
                double gun_yaw_predct = yaw + v_yaw * all_time;    // 预测yaw

                // get armors num
                int armors_num = 4;
                if (aim_id == 11 || aim_id == 12 || aim_id == 13) {
                    armors_num = 2;
                } else if (aim_id == 7) {
                    armors_num = 3;
                }

                cv::Point3d p[armors_num];
                cv::Point3d gunp[armors_num];

                cv::Point2d p_predict_center_2d(p_predict_center.x, p_predict_center.y);
                // cv::Point2d gun_predict_center_2d(gun_predict_center.x, gun_predict_center.y);
                double angle_to_center[armors_num];

                bool is_current_pair = true;
                double r = 0;
                int min_dis_point_index = 0;
                // int min_angle_point_index = 0; 
                double min_dis = DBL_MAX;
                // double min_angle = 1.0;

                // 整车模型建立
                for (int i = 0; i < armors_num; i++)
                {
                    double tmp_yaw = yaw_predict + i * (2.0 * M_PI / armors_num);
                    double gun_temp_yaw = gun_yaw_predct + i * (2.0 * M_PI / armors_num);
                    if (armors_num == 4) {
                        r = is_current_pair ? r1 : r2;
                        gunp[i].z = za + (is_current_pair ? 0 : dz);
                        p[i].z = za + (is_current_pair ? 0 : dz);
                        is_current_pair = !is_current_pair;
                    } else {
                        r = r1;
                        p[i].z = za;
                        gunp[i].z = za;
                    }
                    p[i].x = (p_predict_center.x - r * cos(tmp_yaw));
                    gunp[i].x = (gun_predict_center.x - r * cos(gun_temp_yaw));
                    p[i].y = (p_predict_center.y - r * sin(tmp_yaw));
                    gunp[i].y = (gun_predict_center.y - r * sin(gun_temp_yaw));

                    double dis = sqrt(pow(p[i].x, 2) + pow(p[i].y, 2) + pow(p[i].z, 2));
                    if (armors_num == 3) {
                        if (dis < min_dis) {
                            if (abs(dis - min_dis) > 0) {
                                min_dis = dis;
                                min_dis_point_index = i;        
                            }
                        }
                    } else {
                        if (dis < min_dis) {
                            if (abs(dis - min_dis) > 0.2) {
                                min_dis = dis;
                                min_dis_point_index = i;        
                            }
                        }
                    }

                    cv::Point2d p_2d(p[i].x - p_predict_center_2d.x, p[i].y - p_predict_center_2d.y);
                    angle_to_center[i] = (p_2d.x * p_predict_center_2d.x +
                                          p_2d.y * p_predict_center_2d.y) /
                                         (sqrt(p_2d.x * p_2d.x + p_2d.y * p_2d.y) *
                                          sqrt(p_predict_center_2d.x * p_predict_center_2d.x +
                                               p_predict_center_2d.y * p_predict_center_2d.y));     // 圆心和原点连线，圆心与装甲板连线，两者夹角的余弦值
                    // if (angle_to_center[i] < min_angle)
                    //     min_angle_point_index = i;
                    //     min_angle = angle_to_center[i];
                }

                // 求解最优点
                double limit_area = 0.60 + abs(v_yaw) * 0.05;
                if (limit_area > 0.99)
                    limit_area = 0.99;

                // 前哨站
                if (aim_id == 7)
                {
                    limit_area = 0.73;

                    if (angle_to_center[min_dis_point_index] < -limit_area)
                    {
                        aiming_point = p[min_dis_point_index];
                        // gun_point = p_predict_center;
                        gun_point = gunp[min_dis_point_index];

                        if (angle_to_center[min_dis_point_index] > -0.85)
                            return 2;
                        else
                            return 1;
                    }
                    else
                    {
                        int next_index_1 = (min_dis_point_index + 1) % armors_num;
                        int next_index_2 = min_dis_point_index - 1;
                        if (min_dis_point_index == 0)
                            next_index_2 = armors_num - 1;
                        cv::Point3f next_point_1 = p[next_index_1];
                        cv::Point3f next_point_2 = p[next_index_2];
                        cv::Point3f next_point = next_point_1;
                        cv::Point3f gun_next_point_1 = gunp[next_index_1];
                        cv::Point3f gun_next_point_2 = gunp[next_index_2];
                        cv::Point3f gun_next_point = next_point_1;
                        if (v_yaw < 0) {
                            next_point = next_point_1;
                            gun_next_point = gun_next_point_1;
                        } else {
                            next_point = next_point_2;
                            gun_next_point = gun_next_point_2;
                        }

                        aiming_point = next_point;
                        gun_point = p_predict_center;
                        // gun_point = gun_next_point;

                        return 2;
                    }
                }

                // 转速太快
                if (abs(v_yaw) > 6)
                {
                    center = true;

                    gun_point = p_predict_center;
                    // gun_point = gunp[min_dis_point_index];

                    int next_index_1 = (min_dis_point_index + 1) % armors_num;
                    int next_index_2 = min_dis_point_index - 1;
                    if (min_dis_point_index == 0)
                        next_index_2 = armors_num - 1;
                    cv::Point3f next_point_1 = p[next_index_1];
                    cv::Point3f next_point_2 = p[next_index_2];
                    cv::Point3f next_point = next_point_1;

                    if (v_yaw < 0)
                        next_point = next_point_1;
                    else
                        next_point = next_point_2;

                        aiming_point = p[min_dis_point_index];
                        // aiming_point = next_point;

                    aim2_point = next_point;

                    return 1;
                }

                if (angle_to_center[min_dis_point_index] < -limit_area)
                {
                    aiming_point = p[min_dis_point_index];
                    gun_point = gunp[min_dis_point_index];
                    if (abs(v_yaw) > 2.6)
                        gun_point = p_predict_center;

                    return 1;
                }

                else
                {
                    int next_index_1 = (min_dis_point_index + 1) % armors_num;
                    int next_index_2 = min_dis_point_index - 1;
                    if (min_dis_point_index == 0)
                        next_index_2 = armors_num - 1;
                    cv::Point3f next_point_1 = p[next_index_1];
                    cv::Point3f next_point_2 = p[next_index_2];
                    cv::Point3f next_point = next_point_1;
                    cv::Point3f gun_next_point_1 = gunp[next_index_1];
                    cv::Point3f gun_next_point_2 = gunp[next_index_2];
                    cv::Point3f gun_next_point = next_point_1;
                    if (v_yaw < 0) {
                        next_point = next_point_1;
                        gun_next_point = gun_next_point_1;
                    } else {
                        next_point = next_point_2;
                        gun_next_point = gun_next_point_2;
                    }

                    aiming_point = next_point;
                    gun_point = gun_next_point;
                    if (abs(v_yaw) > 2.6)
                        gun_point = p_predict_center;

                    return 2;
                }
            }
            else if (provirty_list[i+1] != -1)
                continue;
            else
            {
                *attack_id = -1;
                aiming_point = cv::Point3f(0,0,0);
                gun_point = cv::Point3f(0,0,0);

                return 3;
            }
        }
    }

    bool Controler_r::judgeFire(cv::Point3f aiming_point_camera , double v_yaw, cv::Point2f &aim_point_2d_out, bool print, int id)
    {
        cv::Point2f aim_point_2d;
        cv::Mat a;
        a = (cv::Mat_<double>(3, 1) << aiming_point_camera.x, aiming_point_camera.y, aiming_point_camera.z);
        cv::Mat b = camera_matrix_*a;
        cv::Point3d b_new(b);
        b_new = b_new / b_new.z;
        aim_point_2d.x = std::ceil(b_new.x);
        aim_point_2d.y = std::ceil(b_new.y);

        double x = aim_point_2d.x;
        double delta_x = abs(x - this->true_x_);
        // if (print)
            // std::cout << "x  is   " << x << std::endl;

        aim_point_2d_out = aim_point_2d;
        double fire_area = abs(110 / v_yaw);
        if (abs(v_yaw) == 2.512)
            fire_area = camera_matrix_.at<double>(0, 0) * 50 / aiming_point_camera.z;
        else
            fire_area = camera_matrix_.at<double>(0, 0) * 100 / aiming_point_camera.z;
        if (id == 1 || id == 11 || id == 12 || id == 13)
            fire_area = camera_matrix_.at<double>(0, 0) * 140 / aiming_point_camera.z;
        // std::cout << "v_yaw is " << v_yaw << std::endl;                  
        // std::cout << "fire_area is " << fire_area << std::endl;

        if (fire_area < 5) {
            fire_area = 5;
        }
        if (fire_area > 150) {
            fire_area = 150;
        }

        // if (abs(v_yaw) == 2.512 && tracker_[7].tracker_state == base::TEMP_LOST)
        //     return false;

        if (delta_x < fire_area) {
            return true;
        } else {
            return false;
        }
    }

    bool Controler_r::getParam(cv::Mat camera_matrix)
    {
        this->camera_matrix_ = camera_matrix;
        if (camera_matrix.empty()) {
            std::cout << "camera matrix while get param empty" << std::endl;
            return false;
        } else {
            return true;
        }
    }
}
