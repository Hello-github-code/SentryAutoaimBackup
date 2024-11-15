#ifndef RMOS_MIX_DETECTOR_HPP
#define RMOS_MIX_DETECTOR_HPP

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>
#include <iostream>

#include "../../../Base/armor.hpp"

#include "Runtime.hpp"

namespace MixDetect
{
   //! 装甲板限制参数
    struct ArmorParam
    {
        float blue_light_min_threshold=50;
        float red_light_min_threshold=10;
        float gray_light_min_threshold=80;
        int minCounterSize=3;
        int enemy_color = 1;
        void loadParam(const std::string &filename="./src/Algorithm/configure/Detector/detector/mix_detector/ArmorBox.xml")
        {

            cv::FileStorage fs(filename, cv::FileStorage::READ);
            if (!fs.isOpened())
            {
                std::cout << "please check your armor param file" << std::endl;
                assert(fs.isOpened());
                return;
            }
            fs["blue_light_min_threshold"] >> this->blue_light_min_threshold;
            fs["red_light_min_threshold"] >> this->red_light_min_threshold;
            fs["gray_light_min_threshold"] >> this->gray_light_min_threshold;
            fs["minCounterSize"] >> this->minCounterSize;
            fs["enemy_color"] >> this->enemy_color;
            std::cout << "have loaded the armor box params" << std::endl;
        }
        // ArmorParam(const std::string &filename="../Config/params/ArmorBox.xml"){
        //     loadParam();
        // }
    };

    namespace Tool
    {

        inline double getRotateRectAngle(const cv::RotatedRect &rect)
        {
            return rect.size.height > rect.size.width ? -1 * abs(rect.angle) : (rect.angle + 90);
        }
        inline double getRotateRectlength(const cv::RotatedRect &rect)
        {
            return std::max(rect.size.height, rect.size.width);
        }

        inline void getRotateRectPoints(const cv::RotatedRect &rect, cv::Point2d *pt)
        {
            cv::Point2f rect_pt[4];
                rect.points(rect_pt);
            if (rect.size.width > rect.size.height)
            {
                pt[0] = (rect_pt[1] + rect_pt[0]) * 0.5;
                pt[1] = (rect_pt[2] + rect_pt[3]) * 0.5;
            }
            else
            {
                pt[0] = (rect_pt[0] + rect_pt[3]) * 0.5;
                pt[1] = (rect_pt[1] + rect_pt[2]) * 0.5;
            }
        }

        inline void getDoubleRotaRectPt(const cv::RotatedRect &l_rect, const cv::RotatedRect &r_rect, std::vector<cv::Point2d> &pt)
        {
            pt.resize(4);
            cv::Point2d l_pt[2];
            Tool::getRotateRectPoints(l_rect, l_pt);
            cv::Point2d r_pt[2];
            Tool::getRotateRectPoints(r_rect, r_pt);
            pt[0] = l_pt[0];
            pt[1] = l_pt[1];
            pt[2] = r_pt[1];
            pt[3] = r_pt[0];
        }

        inline cv::Rect2d getPtsBBox(std::vector<cv::Point2d> &points)
        {
            double x_min = points.at(0).x, x_max = points.at(0).x;
            double y_min = points.at(0).y, y_max = points.at(0).y;
            for (auto pt : points)
            {
                if (pt.x > x_max)
                {
                    x_max = pt.x;
                }
                else if (pt.x < x_min)
                {
                    x_min = pt.x;
                }
                if (pt.y > y_max)
                {
                    y_max = pt.y;
                }
                else if (pt.y < y_min)
                {
                    y_min = pt.y;
                }
            }
            return cv::Rect2d(cv::Point2d(x_min, y_min), cv::Point2d(x_max, y_max));
        }

        inline cv::Rect2d getDoubleRotaRectBBox(const cv::RotatedRect &l_rect, const cv::RotatedRect &r_rect)
        {
            std::vector<cv::Point2d> pt;
            getDoubleRotaRectPt(l_rect, r_rect, pt);
            return getPtsBBox(pt);
        }
    }

    class Armor
    {
    public:
        cv::Rect2d _bbox;
        int _id;
        int _color;
        std::vector<cv::Point2d> _points;
        bool _if_stable = false;
        Armor(cv::Rect2d &bbox, int id, int color, std::vector<cv::Point2d> &pts)
            : _id(id), _color(color), _bbox(bbox)
        {
            _points.clear();
            _points = std::vector<cv::Point2d>(pts.begin(), pts.end());
        }

        Armor(){};
        ~Armor() = default;

        void transformer(Armor &target, const cv::Point2d &offset = cv::Point2d(0, 0))
        {
            target._id = this->_id;
            target._color = this->_color;
            target._points.clear();
            for (auto pt : this->_points)
            {
                target._points.push_back(pt + offset);
            }
            target._bbox = Tool::getPtsBBox(target._points);
        }
    };

    class LightBlob
    {
    public:
        cv::RotatedRect rect;
        double length;
        double angle;

    public:
        LightBlob(cv::RotatedRect &r) : rect(r)
        {
            length = Tool::getRotateRectlength(r);
            angle = Tool::getRotateRectAngle(r);
        }
        LightBlob() = default;

        ~LightBlob() {}
    };
    typedef std::vector<LightBlob> LightBlobs;


    /********************* 装甲板类定义　************************/
    class ArmorBox
    {
    public:
        cv::Rect2d _rect; //! 装甲板旋转矩形

        LightBlobs _light_Blobs; //! 装甲板的一对灯条

        int _id = -1;

        int _color;

        // 灯条的四个顶点
        std::vector<cv::Point2d> _points; // bl,tl,tr,br;

        ArmorBox(const LightBlob &l_blob, const LightBlob &r_blob, const int pred_id, const int color)
        {
            Tool::getDoubleRotaRectPt(l_blob.rect, r_blob.rect, this->_points);
            this->_rect = Tool::getPtsBBox(this->_points);
            this->_light_Blobs = {l_blob, r_blob};
            _id = pred_id;
            _color = color;
        }

        ArmorBox(){}
        void toArmor(Armor &target, const cv::Point2d &offset = cv::Point2d(0, 0))
        {
            target._id = this->_id;
            target._color = this->_color;
            std::vector<cv::Point2d> pts;
            target._points.clear();
            for (auto pt : this->_points)
            {
                target._points.push_back(pt + offset);
            }
            target._bbox = Tool::getPtsBBox(target._points);
        }

        ~ArmorBox() {}
    };

    typedef std::vector<ArmorBox> ArmorBoxes;


    class ArmorDetector
    {
    private:
        int _color;
        bool is_findBox;

        ArmorParam _armor_param;
        Armor _dl_armor, _target;
        ArmorBox _armor_box;

        cv::Point2d _offset;
        cv::Mat roi;


        // bool findLightBlobs(LightBlobs &light_blobs);

        bool filterLightBlob(const cv::RotatedRect &rect);

        bool matchArmorBox(LightBlobs &light_blobs);

        int  findLightBlobs(cv::Mat img , LightBlobs &light_blobs,cv::Point2d offset);

    public:
        void setParam(ArmorParam &param);

        bool detect(cv::Mat &img, Armor &dl_armor);

        void getResult(Armor &result);

        bool setEnemyColor(int enemy_color);

        ArmorDetector(ArmorParam &param);
        ArmorDetector();
        

        ~ArmorDetector() = default;
        
        std::shared_ptr<ArmorDetector> ptr;
        Runtime rt;
        int enemy_color_;
        cv::Mat debug_binary_;
    };
}
#endif
