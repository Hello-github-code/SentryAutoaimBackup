#ifndef RMOS_NUMBER_CLASSIFIER_HPP
#define RMOS_NUMBER_CLASSIFIER_HPP

#include <opencv2/opencv.hpp>

#include "onnx/onnxruntime_cxx_api.h"
#include "../../../Base/armor.hpp"

namespace detector {
class NumberClassifier {
public:
  NumberClassifier() {
    std::string model_path = "./src/Algorithm/configure/Detector/classifier/new_classifier/model/lenet.onnx";
    net_ = cv::dnn::readNetFromONNX(model_path);
  };
  bool classifyArmors(const cv::Mat &image, std::vector<base::Armor> &armors);

private:
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size{20, 28};
  const cv::Size input_size{28, 28};

  cv::Mat number_image;
  cv::dnn::Net net_;
  std::mutex mutex_;
};
}  // namespace detector

#endif  //RMOS_NUMBER_CLASSIFIER_HPP
