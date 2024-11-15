//
// Created by Wang on 23-7-15.
//


#include "Tool/tracker/tracker.hpp"



namespace tool
{
    Tracker::Tracker()
            : tracker_state(base::LOST),
              tracked_id(-1),
              move_measurement(Eigen::VectorXd::Zero(4)),
              rotate_measurement(Eigen::VectorXd::Zero(1)),
              move_target_state(Eigen::VectorXd::Zero(8)),
               rotate_target_state(Eigen::VectorXd::Zero(2))

    {
        cv::FileStorage fs("./src/Algorithm/configure/Tool/tracker/param.xml", cv::FileStorage::READ);

        if(!fs.isOpened())
        {
            std::cout<<"open tool tracker param fail"<<std::endl;
            exit(0);
        }

        fs["max_match_distance"] >> max_match_distance_;
        fs["max_match_yaw_diff"] >> max_match_yaw_diff_;
        fs["tracking_thres"] >> tracking_thres;
        fs["lost_thres"] >> lost_thres;

        fs.release();


    }


    void Tracker::init(std::vector<base::Armor> armors)
    {
        if (armors.empty()) {
            return;
        }

        double min_distance = DBL_MAX;
        tracked_armor = armors[0];
        for (const auto &armor: armors) {
            if (armor.distance_to_image_center < min_distance) {
                min_distance = armor.distance_to_image_center;
                tracked_armor = armor;
            }
        }

        initEKF(tracked_armor);

        tracked_id = tracked_armor.num_id;
        tracker_state = base::DETECTING;

    }


    void Tracker::update(std::vector<base::Armor> armors)
    {
        // KF predict
        Eigen::VectorXd move_ekf_prediction = moveekf.predict(rotate_target_state(1));
        Eigen::VectorXd rotate_ekf_prediction = rotateekf.predict();
        
        bool matched = false;
        // Use KF prediction as default target state if no matched armor is found
        move_target_state = move_ekf_prediction;
        rotate_target_state = rotate_ekf_prediction;
        //std::cout<<"laleyaw="<<rotate_target_state(0)<<'\n';
            int same_id_armors_count = 0;
           double center_x_diff=0;
           last_armor_size=0;

        //std::vector<bool> tempobstate;       
        if (!armors.empty())
         {
        //    for (const auto & armor : armors)
        //   {
        //     if (armor.num_id == tracked_id)
        //     {
        //         if(armor.leftgap==1)
        //         tempobstate.push_back(0);
        //         tempobstate.push_back(1);
        //         if(armor.rightgap==1)
        //         tempobstate.push_back(0);
                
        //     }

        //   }
          
        //         int  jumped=0;

        //   if(last_obstate==std::vector<bool>{1,0,1}&&tempobstate==std::vector<bool>{0,1,0})
        //  jumped=1;
        //  else if(last_obstate==std::vector<bool>{1,0,0,1}&&tempobstate==std::vector<bool>{0,1})
        //  jumped = 2;//left
        // else if(last_obstate==std::vector<bool>{1,0,0,1}&&tempobstate==std::vector<bool>{1,0})
        //  jumped = 3;//right
        // else if(last_obstate==std::vector<bool>{0,1,0}&&tempobstate==std::vector<bool>{0,1})
        //  jumped = 2;//left
        // else if(last_obstate==std::vector<bool>{0,1,0}&&tempobstate==std::vector<bool>{1,0})
        //  jumped = 3;//right
        //  else if(last_obstate==std::vector<bool>{0,1}&&tempobstate==std::vector<bool>{1,0})
        //  jumped = 3;//right
        //   else if(last_obstate==std::vector<bool>{1,0}&&tempobstate==std::vector<bool>{0,1})
        //  jumped = 2;//left

        //  last_obstate=tempobstate;

        //  if(tracked_id == 7)
        //  {
        //     if(jumped ==2)
        //     outpost_direction=-1;
        //     else if(jumped = 3)
        //     outpost_direction=1;
        //  }






            
            // Find the closest armor with the same id
            base::Armor same_id_armor;
            auto predicted_position = getArmorPositionFromState(move_ekf_prediction,rotate_ekf_prediction);
            double min_position_diff = DBL_MAX;
            double yaw_diff = DBL_MAX;

            for (const auto & armor : armors) {
                // Only consider armors with the same id
                if (armor.num_id == tracked_id) {
                    same_id_armor = armor;
                    same_id_armors_count++;
                    // Calculate the difference between the predicted position and the current armor position
                    auto p = armor.position;
                    Eigen::Vector3d position_vec(p.x, p.y, p.z);
                    double position_diff = (predicted_position - position_vec).norm();
                    if (abs(armor.yaw -this->last_yaw_) < abs(yaw_diff)) {
                        // Find the closest armor
                        min_position_diff = position_diff;
                        yaw_diff = armor.yaw -this->last_yaw_;
                        tracked_armor = armor;
                                      // ??
                    }
                }
            }

         
            
           center_x_diff=tracked_armor.center_point.x-last_center_x;

            // Check if the distance and yaw difference of closest armor are within the threshold
             if(abs(rotate_target_state(1))>5.2&&abs(yaw_diff)<0.035)
                 {
                    vyaw_over_count++;
                 }
                 else
                 {
                    vyaw_over_count=0;
                 }
           
           if(last_state==2)
           {
               matched = true;
                //std::cout<<"jump"<<'\n';
               //std::cout<<"jump="<<same_id_armor.yaw/3.1415926*180<<" with vyaw="<<rotate_target_state(1)<<'\n';
                                 

                handleArmorJump(same_id_armor);
                 this->last_yaw_ = same_id_armor.yaw; 
                 this->last_center_x = same_id_armor.center_point.x;
                 last_state=0;
           }
           else if ((last_armor_size==0&&same_id_armors_count == 1 && abs(yaw_diff) > 45.0/180.0*3.1415926&&tracked_armor.num_id!=7)
           ||(last_armor_size==2&&same_id_armors_count==1&&rotate_target_state(1)>0.1&&yaw_diff<-0.4&&tracked_armor.num_id!=7)
           ||(last_armor_size==2&&same_id_armors_count==1&&rotate_target_state(1)<-0.1&&yaw_diff>0.4&&tracked_armor.num_id!=7)
           ||(last_armor_size==0&&same_id_armors_count==1&&rotate_target_state(1)>0.1&&yaw_diff<-0.5&&tracked_armor.num_id!=7)
           ||(last_armor_size==0&&same_id_armors_count==1&&rotate_target_state(1)<-0.1&&yaw_diff>0.5&&tracked_armor.num_id!=7)
           ||(same_id_armors_count == 1 && abs(yaw_diff) > 30.0/180.0*3.1415926&&tracked_armor.num_id!=7)
           ||(rotate_target_state(1)>2.5&&yaw_diff<-rotate_target_state(1)*0.003&&tracked_armor.num_id!=7)
           ||(rotate_target_state(1)<-2.5&&yaw_diff>-0.003*rotate_target_state(1)&&tracked_armor.num_id!=7)
           //||(abs(rotate_target_state(1))>2.5&&abs(yaw_diff)<0.03&&vyaw_over_count<6&&tracked_armor.num_id!=7)
           )
           {
                // Matched armor not found, but there is only one armor with the same id
                // and yaw has jumped, take this case as the target is spinning and armor jumped
                matched = true;
                //std::cout<<"jump"<<'\n';
               //std::cout<<"jump="<<same_id_armor.yaw/3.1415926*180<<" with vyaw="<<rotate_target_state(1)<<'\n';
                last_state =2;
                handleArmorJump(same_id_armor);
                this->last_yaw_ = same_id_armor.yaw; 
                this->last_center_x = same_id_armor.center_point.x;
                
            } 
            else if (min_position_diff < 10*max_match_distance_ && abs(yaw_diff) < 3.1415926) {
                // Matched armor found
                matched = true;
                last_state =1;
                auto p = tracked_armor.position;
                // Update EKF
                double measured_yaw = tracked_armor.yaw;
                move_measurement = Eigen::Vector4d(p.x, p.y, p.z,measured_yaw);
                 Eigen::VectorXd m_yaw(1);
                 m_yaw<<measured_yaw;
                rotate_measurement =m_yaw;
                //std::cout<<"match="<<m_yaw/3.1415926*180<<" with vyaw="<<rotate_target_state(1)<<'\n';
                move_target_state = moveekf.update(move_measurement,move_target_state(1));
               // std::cout<<"update with"<<p.x<<" "<< p.y<<'\n';

               rotate_target_state = rotateekf.update(rotate_measurement);
               //  std::cout<<"v_yawm="<<rotate_target_state(1)<<'\n';
                this->last_yaw_ = tracked_armor.yaw;
                               //std::cout<<"match"<<'\n';
                 this->last_center_x = same_id_armor.center_point.x;

            }
            else 
            {
                matched = false;
                // No matched armor found
                                 this->last_center_x = same_id_armor.center_point.x;
                this->last_yaw_ = tracked_armor.yaw;
                last_state=3;

            }
            last_armor_size=same_id_armors_count;
        }

        // Suppress R from spreading
        if( tracked_id == 1)
        {
            if (move_target_state(7) < 0.3) {
                move_target_state(7) = 0.3;
                moveekf.setState(move_target_state);
            }else if(move_target_state(7) > 0.4)
            {
                move_target_state(7) = 0.4;
                moveekf.setState(move_target_state);     
            }

        }
        else
        {
            if (move_target_state(7) < 0.18) {
                move_target_state(7) = 0.18;
                moveekf.setState(move_target_state);
            }else if(move_target_state(7) > 0.3){
                move_target_state(7) = 0.3;
                moveekf.setState(move_target_state);
            }
              
        }
        if(rotate_target_state(1)>14)
            rotate_target_state(1) =0;
        else if(rotate_target_state(1)<-14)
            rotate_target_state(1)=0;
            
        if(move_target_state(1)>3)
            move_target_state(1)=3;
        else if(move_target_state(1)<-3)
            move_target_state(1)=-3;

        if(move_target_state(3)>3)
            move_target_state(3)=3;
            
        else if(move_target_state(3)<-3)
            move_target_state(3)=-3;
       //  std::cout<<"nb"<<move_target_state(7)<<'\n';
        if(tracked_id == 7) 
        {
             if(rotate_target_state(1)>0.2&&outpost_direction==1)
            {
               rotate_target_state(1) = 2.512;
               outpost_direction_ = 1;
            }
            else if(rotate_target_state(1)<-0.2&&outpost_direction==-1)
            {
              rotate_target_state(1) = -2.512;
              outpost_direction_ = -1;
            }
            if(last_armor_size==1&&same_id_armors_count==1&&center_x_diff<-5)
            outpout_direction_count--;
            else if(last_armor_size==1&&same_id_armors_count==1&&center_x_diff>5)
            outpout_direction_count++;

            if(outpout_direction_count>=25)
            {
               rotate_target_state(1) = 2.512;
               outpost_direction_ = 1;
            }
            else if(outpout_direction_count<=-25)
            {
              rotate_target_state(1) = -2.512;
              outpost_direction_ = -1;
            }
            move_target_state(7) = 0.2712;
            move_target_state(1) = 0;
            move_target_state(3) = 0;
            move_target_state(5) = 0;
            
            for(const auto & armor : armors)
            {
                if(armor.num_id==7&&armor.k_<0.1&&abs(armor.points[0].y-armor.points[3].y)<3)
                {
                    std::cout<<armor.k_<<'\n';
                    cv::Point3f s_center;
                    double angle_t=atan2(armor.position.y,armor.position.x);
                    float r=sqrt(armor.position.x*armor.position.x+armor.position.y*armor.position.y);
                    s_center.x=(r+0.2712)*cos(angle_t);
                    s_center.y=(r+0.2712)*sin(angle_t);
                    move_target_state(0)=s_center.x;
                    move_target_state(2)=s_center.y;
                    rotate_target_state(0)=angle_t;
                    std::cout<<"sss"<<abs(angle_t-armor.yaw)/3.1415*180<<'\n';
                }
            }
            moveekf.setState(move_target_state);
            rotateekf.setState(rotate_target_state);
            
            
        }

        if(abs(dz)>0.055)
        {
            if(dz>0)
            {
                dz = 0.055;
            }
            else
            {
                dz = -0.055;
            }
        }
        ////
        if(abs(another_r- move_target_state(7))>0.15)
        {
            double average_r = (another_r + move_target_state(7))/2;
            move_target_state(7) = average_r;
            another_r = average_r;
        }

        if(abs(move_target_state(5))>0)
        {
            move_target_state(5) = 0;
        }
            move_target_state(7) = 0.2;



        // Tracking state machine
        if (tracker_state == base::DETECTING) {
            if (matched) {
                detect_count_++;
                if (detect_count_ > tracking_thres) {
                    detect_count_ = 0;
                    tracker_state = base::TRACKING;
                }
            } else {
                tracker_state = base::LOST;
            }
        } else if (tracker_state == base::TRACKING) {
            if (!matched) {
                tracker_state = base::TEMP_LOST;
                lost_count_++;
            }
        } else if (tracker_state == base::TEMP_LOST) {
            if (!matched) {
                lost_count_++;
               // std::cout<<"loos::" << lost_thres << "lo::" << lost_count_ <<"\n";

                if (lost_count_ > lost_thres) {
                    lost_count_ = 0;
                    tracker_state = base::LOST;
                }
            } else {
                tracker_state = base::TRACKING;
                lost_count_ = 0;
            }
        }
    }

    void Tracker::reset()
    {
        this->tracker_state = base::LOST;
        this->lost_count_ = 0;
        this->detect_count_ = 0;
        this->tracked_id = -1;
    }












    void Tracker::initEKF(const base::Armor & a)
    {
        double xa = a.position.x;
        double ya = a.position.y;
        double za = a.position.z;
        last_yaw_ = 0;
        double yaw = a.yaw;
        last_yaw_ = a.yaw;

        // Set initial position at 0.26m behind the target
        move_target_state = Eigen::VectorXd::Zero(8);
        double r = 0.18;
        double xc = xa + r * cos(yaw);
        double yc = ya + r * sin(yaw);
        double zc = za;
        dz = 0, another_r = r;
        move_target_state << xc, 0, yc, 0, za, 0, yaw, r;
        //std::cout<<"initekfwith "<<xc<<" " <<yc<<" "<<xa<<" "<<ya<<'\n';
        rotate_target_state = Eigen::VectorXd::Zero(2);
        rotate_target_state << yaw,0;
        moveekf.setState(move_target_state);
        rotateekf.setState(rotate_target_state);
    }

    void Tracker::handleArmorJump(const base::Armor & a)
    {
        double yaw = a.yaw;
        move_target_state(6) = yaw;
        rotate_target_state(0) = yaw;

        // Only 4 armors has 2 radius and height
        if (a.num_id != 11 && a.num_id != 12 && a.num_id != 13 && a.num_id != 7) {
            dz = move_target_state(4) - a.position.z;
            move_target_state(4) = a.position.z;
            std::swap(move_target_state(7), another_r);
        }

        // If position difference is larger than max_match_distance_,
        // take this case as the ekf diverged, reset the state
        auto p = a.position;
        Eigen::Vector3d current_p(p.x, p.y, p.z);
        Eigen::Vector3d infer_p = getArmorPositionFromState(move_target_state,rotate_target_state);
        if ((current_p - infer_p).norm() > 0.57&& a.num_id!=7) {
            double r = move_target_state(7);
            move_target_state(0) = p.x + r * cos(yaw);  // xc
            move_target_state(1) = move_target_state(1);                   // vxc
            move_target_state(2) = p.y + r * sin(yaw);  // yc
            move_target_state(3) = move_target_state(3);                   // vyc
            move_target_state(4) = p.z;                 // za
            move_target_state(5) = 0;                   // vza
            this->last_yaw_ = yaw;   
             
            //std::cout<<"cover "<<(current_p - infer_p).norm()<<'\n';             // ??
        }
        
       // std::cout<<"v_yawj="<<rotate_target_state(1)<<'\n';
        moveekf.setState(move_target_state);
        rotateekf.setState(rotate_target_state);

    }

    Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd & mx,const Eigen::VectorXd & rx)
    {
        // Calculate predicted position of the current armor
        double xc = mx(0), yc = mx(2), za = mx(4);
        double yaw = rx(0), r = mx(7);
        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);
        return Eigen::Vector3d(xa, ya, za);
    }






}