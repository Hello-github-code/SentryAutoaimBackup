#include "Dectector/classifier/new_classifier/number_classifier.hpp"

#include <cmath>
#include <utility>

namespace detector {
bool NumberClassifier::classifyArmors(const cv::Mat &image, std::vector<base::Armor> &armors) {
  for (auto &armor : armors) {
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {armor.left.down, armor.left.up, armor.right.up,
                                      armor.right.down};

    const int top_light_y = (this->warp_height - this->light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + this->light_length;
    const int warp_width =
      armor.type == base::ArmorType::SMALL ? this->small_armor_width : this->large_armor_width;
    cv::Point2f target_vertices[4] = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(image, this->number_image, rotation_matrix,
                        cv::Size(warp_width, warp_height));

    // Get ROI
    this->number_image =
      this->number_image(cv::Rect(cv::Point((warp_width - this->roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(this->number_image, this->number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(this->number_image, this->number_image, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::resize(this->number_image, this->number_image, this->input_size);

    // cv::imshow("num", this->number_image);
    // cv::waitKey(1);
    this->number_image /= 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(this->number_image, blob);

    // Set the input blob for the neural network
    this->mutex_.lock();
    net_.setInput(blob);

    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward().clone();
    this->mutex_.unlock();

    // Decode the output
    double confidence;
    cv::Point class_id_point;
    minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = confidence;
    armor.num_id = label_id + 1;
    // std::cout << "label_id: " << armor.num_id
    //           << " type: " << (base::ArmorType::SMALL == armor.type ? "SMALL" : "LARGE")
    //           << std::endl;

    if (label_id >= 9) {
      armor.type = base::ArmorType::WRONG;
    }

    // double confidence = 0;
    // armor.num_id = this->classifyArmor(temp,confidence);
    // armor.confidence = confidence;
    switch (armor.num_id) {
      case 7:
        armor.num_id = 6;  //哨兵
        armor.type = base::ArmorType::SMALL;
        break;
      case 1:
        armor.type = base::ArmorType::BIG;
        break;
      case 2:
        armor.type = base::ArmorType::SMALL;
        break;
      case 6:
        armor.num_id = 7;  //前哨站
        armor.type = base::ArmorType::SMALL;
        break;
      case 9:
        armor.type = base::ArmorType::WRONG;
        break;
    }
  }
  armors.erase(std::remove_if(armors.begin(), armors.end(),
                              [this](const base::Armor &armor) {
                                if (armor.type == base::ArmorType::WRONG) {
                                  return true;
                                } else {
                                  return false;
                                }
                              }),
               armors.end());

  return true;
}

}  // namespace detector