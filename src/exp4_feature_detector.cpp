#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include "feature_detector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "exp4_image_subscriber");
  std::cout << "===node exp4_image_subscriber starts===" << std::endl;

  lec4::FeatureDetector sub;

  ros::spin();
  return 0;
}