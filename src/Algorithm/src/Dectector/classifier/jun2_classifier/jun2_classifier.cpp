#pragma once

#include "Dectector/classifier/jun2_classifier/jun2_classifier.hpp"
// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace detector {

NumberClassifier::NumberClassifier()
: threshold(0.5)
{
  net_ = cv::dnn::readNetFromONNX("./src/Algorithm/configure/Detector/classifier/jun2_classifier/model/mlp.onnx");
  ignore_classes_.emplace_back("negative");
  std::ifstream label_file("./src/Algorithm/configure/Detector/classifier/jun2_classifier/model/label.txt");
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
  for (int i = 0; i < 256; i++)
  {
    float normalize = (float)(i/255.0);
    lut[i] = cv::saturate_cast<uchar>(pow(normalize, 0.6) * 255.0f);
  }
}

void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<base::Armor> & armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);
  for (auto & armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.left.down, 
      armor.left.up, 
      armor.right.up,
      armor.right.down
    };

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.type == base::ArmorType::SMALL ? small_armor_width : large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };

    double box_height_enlarge = 1.0; // 这里的参数暂时先直接放在函数里
    armor.rect.y -= armor.rect.height / 2.0 * box_height_enlarge;
    armor.rect.height *= 2.0 * box_height_enlarge;
    armor.rect &= cv::Rect2d(cv::Point(0, 0), src.size());
    // temp = image(cv::Rect2d(armor.rect));
    // for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++)
    //     {
    //         (*it)[0] = lut[((*it)[0])];
    //         (*it)[1] = lut[((*it)[1])];
    //         (*it)[2] = lut[((*it)[2])];
    //     }
   for (int i = armor.rect.x; i < armor.rect.width; i++) {
    for (int j = armor.rect.y; j < armor.rect.height; j++) {
      cv::Vec3b vc = src.at<cv::Vec3b>(i, j);
            vc[0] = lut[(vc[0])];
            vc[1] = lut[(vc[1])];
            vc[2] = lut[(vc[2])];
    }
   }

  cv::Mat number_image;
  auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
  cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));
//////////////
    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat image = number_image.clone();

    image = image / 255.0;

    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);
//////////////////////////////
    // Set the input blob for the neural network
    net_.setInput(blob);
    //std::cout << "what//////////////////////////////////////////////////////" << '\n';
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.number = class_names_[label_id];
   // std::cout << armor.number << '\n';
    armors.erase(
    std::remove_if(
      armors.begin(), armors.end(),
      [this](const base::Armor & armor) {
        if (armor.confidence < threshold) {
          return true;
        }

        for (const auto & ignore_class : ignore_classes_) {
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false;
        if (armor.type == base::ArmorType::BIG) {
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";
        } else if (armor.type == base::ArmorType::SMALL) {
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type;
      }),
    armors.end());
  }
for (auto & armor : armors) {
  if(armor.type==base::ArmorType::BIG)
  {
    if(armor.number=="1")
      {armor.num_id =1;
      continue;}
    if(armor.number=="3")
      {armor.num_id =11;
      continue;}
    if(armor.number=="4")
      {armor.num_id =12;
      continue;}
    if(armor.number=="5")
      {armor.num_id =13;
      continue;}
    if(armor.number=="base")
      {armor.num_id =0;
      continue;}

    }

  else if(armor.type==base::ArmorType::SMALL)
  {
    if(armor.number=="3")
      {armor.num_id =3;
      continue;}
    if(armor.number=="4")
      {armor.num_id =4;
      continue;}
    if(armor.number=="5")
      {armor.num_id =5;
      continue;}
    if(armor.number=="2")
      {armor.num_id =2;
      continue;}
    if(armor.number=="guard")
      {armor.num_id =6;
      continue;}
    if(armor.number=="outpost")
      {armor.num_id =7;
      continue;}

    }
  }
}

}  // namespace rm_auto_aim
