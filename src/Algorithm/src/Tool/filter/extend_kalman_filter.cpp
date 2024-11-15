//
// Created by nuc12 on 23-7-12.
//

#include "Tool/filter/extend_kalman_filter.hpp"
#include<iostream>

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
    {
    }

    void ExtendedKalmanFilter::setState(const Eigen::VectorXd & x0) { x_post = x0; }

    Eigen::MatrixXd ExtendedKalmanFilter::predict(float vyaw)
    {
        
        F = jacobian_f(x_post), Q = update_Q();
        
        //Q(0,0)/=pow(40,abs(vyaw));
       // Q(0,1)/=pow(40,abs(vyaw));
        //Q(1,0)/=pow(40,abs(vyaw));
       // Q(3,2)/=pow(40,abs(vyaw));
       // Q(2,3)/=pow(40,abs(vyaw));
        //Q(2,2)/=pow(40,abs(vyaw));
        
        if(abs(vyaw)>3)
        {Q(3,3)/=pow(4.6,abs(vyaw));
        Q(1,1)/=pow(4.6,abs(vyaw));}
        else
       { Q(3,3)/=(1+2250*abs(vyaw));
        Q(1,1)/=(1+2250*abs(vyaw));}
        if(abs(dvx)<0.1&&abs(dvy)<0.1)
        {Q(1,1)*=100*(0.009+10*abs(dvx));
        Q(3,3)*=100*(0.009+10*abs(dvy));}
        if(abs(vyaw)<0.5)
         {Q(3,3)*=50;
        Q(1,1)*=50;}

        //std::cout<<dvx<<'\n';
        //std::cout<<dvy<<'\n';

        x_pri = f(x_post);
        P_pri = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict
        x_post = x_pri;
        P_post = P_pri;
        last_vx=x_pri(1,0);
        last_vy=x_pri(3,0);

        return x_pri;
    }
    Eigen::MatrixXd ExtendedKalmanFilter::predict()
    {
     
        F = jacobian_f(x_post), Q = update_Q();
        Q(1,1)/=pow(1.45,abs(x_post(1,0)));

        x_pri = f(x_post);
        P_pri = F * P_post * F.transpose() + Q;

        // handle the case when there will be no measurement before the next predict
        x_post = x_pri;
        P_post = P_pri;


        return x_pri;
    }

    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z)
    {
        H = jacobian_h(x_pri), R = update_R(z);

        K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
        x_post = x_pri + K * (z - h(x_pri));
        P_post = (I - K * H) * P_pri;
    //std::cout<<"ekfupdatinf?"<<'\n';
        return x_post;//
    }
    Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd & z,double vyaw)
    {
        H = jacobian_h(x_pri), R = update_R(z);
          if(abs(vyaw)>2)
        {R(0,0)*=10;
        R(0,1)*=10;
        R(0,2)*=10;
        }      
        if(abs(vyaw)>5)
        {R(0,0)*=50;
        R(0,1)*=50;
        R(0,2)*=50;
        }
        K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
        x_post = x_pri + K * (z - h(x_pri));
        P_post = (I - K * H) * P_pri;
    //std::cout<<"ekfupdatinf?"<<'\n';
        dvx=x_post(1,0)-last_vx;
        dvy=x_post(3,0)-last_vy;
        if(abs(dvx)>0.8)
        x_post(1,0)=last_vx;
        if(abs(dvy)>0.8)
        x_post(3,0)=last_vy;
        last_vx=x_post(1,0);
        last_vy=x_post(3,0);
        return x_post;//
    }


}