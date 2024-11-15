#include "Dectector/detector/mix_detector/mix_detector.hpp"

namespace MixDetect
{
    /**
     * @brief   构造检测器
     * @param  param_filename  分类器参数地址
     */
    ArmorDetector::ArmorDetector(ArmorParam &param)
    {
        _armor_param = param;
    }

    ArmorDetector::ArmorDetector()
    {
        rt.init("./src/Algorithm/configure/Detector/detector/mix_detector/rct_rm_v7_run2.yaml");
        std::cout<<rt.init_messages()<<std::endl;
        _armor_param.loadParam();
        setEnemyColor(_armor_param.enemy_color);
    }

    bool ArmorDetector::detect(cv::Mat &img, Armor &dl_armor)
    {

        this->_offset = dl_armor._bbox.tl();
        this->roi = img(dl_armor._bbox);
        this->_color = dl_armor._color;    //0---blue,  1--red
        
        dl_armor.transformer(this->_dl_armor, cv::Point2d(0, 0) - this->_offset);
        is_findBox = false;
        if (this->_color < 2)
        {

            double hight_ratio=1.7;
            double left_ratio=1.1;
            LightBlobs left_light_blobs, right_light_blobs;
            cv::Rect2d left_rect, right_rect;
            cv::Point2d left_center = (_dl_armor._points.at(0) + _dl_armor._points.at(1)) / 2;
            double left_hight = hight_ratio * cv::norm(_dl_armor._points.at(0) - _dl_armor._points.at(1));
            double left_width = left_ratio * cv::norm(_dl_armor._points.at(0) - _dl_armor._points.at(1));
            cv::Point2d left_temp = cv::Point2d(0.5 * left_width, 0.5 * left_hight);
            left_rect = cv::Rect2d(left_center - left_temp, left_center + left_temp);
            left_rect = left_rect&cv::Rect2d(0,0,this->roi.cols,this->roi.rows);
            int left_size = findLightBlobs(this->roi(left_rect), left_light_blobs, left_rect.tl());
            //
            cv::Point2d right_center = (_dl_armor._points.at(2) + _dl_armor._points.at(3)) / 2.0;
            double right_hight = hight_ratio * cv::norm(_dl_armor._points.at(2) - _dl_armor._points.at(3));
            double right_width = left_ratio * cv::norm(_dl_armor._points.at(2) - _dl_armor._points.at(3));
            cv::Point2d right_temp = cv::Point2d(0.5 * right_width, 0.5 * right_hight);
            right_rect = cv::Rect2d(right_center - right_temp, right_center + right_temp);
            right_rect=right_rect&cv::Rect2d(0,0,this->roi.cols,this->roi.rows);
            int right_size = findLightBlobs(this->roi(right_rect), right_light_blobs, right_rect.tl());

            // cv::namedWindow("1",cv::WINDOW_NORMAL);
            // cv::namedWindow("2",cv::WINDOW_NORMAL);
            // cv::imshow("1",this->roi(left_rect));
            // cv::imshow("2",this->roi(right_rect));
            // cv::waitKey(0); 
            // std::cout<<"size  "<<left_size<<" "<<right_size<<std::endl;
            if (left_size == 1 && right_size == 1)
            {
                LightBlobs light_blobs;
                light_blobs.push_back(left_light_blobs.at(0));
                light_blobs.push_back(right_light_blobs.at(0));
                if (matchArmorBox(light_blobs))
                {
                    is_findBox = true;
                }
            }

            // if (findLightBlobs(light_blobs))
            // {
            //     if (matchArmorBox(light_blobs))
            //     {
            //         is_findBox = true;
            //     }
            // }
        }
        if (is_findBox)
        {
            this->_armor_box.toArmor(_target);
        }
        else
        {
            _target = _dl_armor;
        }

        return is_findBox;
    }

    void ArmorDetector::setParam(ArmorParam &param)
    {
        this->_armor_param.loadParam();

    }

    void ArmorDetector::getResult(Armor &result)
    {
        this->_target.transformer(result, this->_offset);
        result._if_stable = is_findBox;
    }

    /**
     * @brief   筛选灯条
     * @param   contours 灯条轮廓
     * @param   rect 灯条的旋转矩形
     */
    bool ArmorDetector::filterLightBlob(const cv::RotatedRect &rect)
    {
        // legth raito filter
        double left_length = cv::norm(_dl_armor._points.at(0) - _dl_armor._points.at(1));
        double right_length = cv::norm(_dl_armor._points.at(3) - _dl_armor._points.at(2));
        double mean_lenght = (left_length + right_length) / 2.0;
        double rect_height = Tool::getRotateRectlength(rect);
        double ratio = mean_lenght > rect_height ? mean_lenght / rect_height : rect_height / mean_lenght;
        if (abs(ratio) > 3)
        {
           // std::cout<<" [filter] ratio "<<std::endl;
            return false;
        }
        // angle filter
        double rect_angle = Tool::getRotateRectAngle(rect);
        double left_angle = (180 / 3.1415926) * atan2((_dl_armor._points.at(1).y - _dl_armor._points.at(0).y), (_dl_armor._points.at(1).x - _dl_armor._points.at(0).x)) - 90;
        double right_angle = (180 / 3.1415926) * atan2((_dl_armor._points.at(2).y - _dl_armor._points.at(3).y), (_dl_armor._points.at(2).x - _dl_armor._points.at(3).x)) - 90;
        double mean_angle = (left_angle + right_angle) / 2.0;
        if (abs(rect_angle - mean_angle) > 20)
        {
           // std::cout<<" [filter] angle "<<std::endl;
            return false;
        }

        return true;
    }

    int ArmorDetector::findLightBlobs(cv::Mat img, LightBlobs &light_blobs, cv::Point2d offset)
    {
        light_blobs.clear();
        if (img.empty())
            return false;
        
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, gray, _armor_param.gray_light_min_threshold, 255, cv::THRESH_OTSU);

        // 分离RGB通道
        std::vector<cv::Mat> bgr_channel;
        cv::split(img, bgr_channel);
        cv::Mat binary_color_img_, binary_Mat;
        if (1 == this->_color)
        {
            // 红通道减蓝通道相减并二值化
            cv::subtract(bgr_channel[2], bgr_channel[0], binary_color_img_);
            cv::threshold(binary_color_img_, binary_color_img_, _armor_param.red_light_min_threshold, 255, cv::THRESH_BINARY);
            binary_Mat = binary_color_img_ & gray;
        }
        else
        {
            cv::subtract(bgr_channel[0], bgr_channel[2], binary_color_img_);
            cv::threshold(binary_color_img_, binary_color_img_, _armor_param.blue_light_min_threshold, 255, cv::THRESH_BINARY);
            binary_Mat = binary_color_img_ & gray;
        }
        cv::Mat elem_COLSE = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(binary_Mat, binary_Mat, cv::MORPH_CLOSE, elem_COLSE);

        std::vector<std::vector<cv::Point>> contours_light;
        cv::findContours(binary_Mat, contours_light, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,offset);
        debug_binary_ = binary_Mat;

        for (auto contour : contours_light)
        {
            if (contour.size() < _armor_param.minCounterSize)
            {
                continue;
            }
            cv::RotatedRect light = cv::minAreaRect(contour);
            if (filterLightBlob(light))
            {
                light_blobs.emplace_back(light);
            }
        }
        // if(light_blobs.size()==0){
        //     cv::namedWindow("1",cv::WINDOW_NORMAL);
        //     cv::imshow("1",img);
        //     cv::namedWindow("2",cv::WINDOW_NORMAL);
        //     cv::imshow("2",gray);
        //     cv::namedWindow("3",cv::WINDOW_NORMAL);
        //     cv::imshow("3",binary_color_img_);
        //     cv::namedWindow("4",cv::WINDOW_NORMAL);
        //     cv::imshow("4",binary_Mat);
        //     cv::waitKey(0);
        // }
        // cv::waitKey(0);
        sort(light_blobs.begin(), light_blobs.end(), [](const LightBlob &a, const LightBlob &b)
             { return a.rect.center.x < b.rect.center.x; });

        return light_blobs.size();
    }

    /**
     * @brief  匹配所有灯条，得出伪装甲板集
     * @param  light_blobs 筛选后得到的灯条集
     * @param  armor_boxes 得到的装甲板集
     */
    bool ArmorDetector::matchArmorBox(LightBlobs &light_blobs)
    {
        if (light_blobs.size() == 2)
        {
            if((light_blobs[0].length / light_blobs[1].length) < 0.8 || (light_blobs[0].length / light_blobs[1].length) > 1.2)
                return false;
            // std::cout << light_blobs[0].length << " " << light_blobs[1].length << std::endl;
            this->_armor_box = ArmorBox(light_blobs[0], light_blobs[1], this->_dl_armor._id, this->_color);
            return true;
        }
        else
        {
            // std::cout << "Multiple amors " << std::endl;
            cv::Rect2d a = Tool::getPtsBBox(_dl_armor._points);
            int idex = -1;
            double iou = -100;
            for (int i = 0; i < light_blobs.size() - 1; i++)
            {
                cv::Rect2d b = Tool::getDoubleRotaRectBBox(light_blobs.at(i).rect, light_blobs.at(i + 1).rect);
                cv::Rect2d c = a & b;
                double center_dis = cv::norm(cv::Point2d(a.x - b.x, a.y - b.y));
                double c_dis = cv::norm(c.br() - c.tl());
                double temp_iou = c.area() / (a.area() + b.area() - c.area());
                // std::cout << "ratio  " << center_dis / b.width << std::endl;
                if (temp_iou > iou)
                {
                    if (center_dis / b.width < 10)
                    {
                        iou = temp_iou;
                        idex = i;
                    }
                }
            }
            if (idex >= 0)
            {
                if((light_blobs[idex].length / light_blobs[idex+1].length) < 0.8 || (light_blobs[idex].length / light_blobs[idex+1].length) > 1.2)
                return false;
                // std::cout << light_blobs[idex].length << " " << light_blobs[idex+1].length << std::endl;

                this->_armor_box = ArmorBox(light_blobs[idex], light_blobs[idex + 1], this->_dl_armor._id, this->_color);
                return true;
            }
        }

        return false;
    }

    bool ArmorDetector::setEnemyColor(int enemy_color)
    {
        // this->process_params_.enemy_color = enemy_color;
        if (enemy_color == 0)
        {
            this->enemy_color_ = base::Color::RED;
            return true;
        }
        else if (enemy_color == 1)//_armor_param
        {
            this->enemy_color_ = base::Color::BLUE;
            return true;
        }
        else
        {
            return false;
        }
    }

}

