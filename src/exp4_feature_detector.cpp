#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>

namespace lec4 {

class FeatureDetector {
 public:
  using Features = std::vector<cv::Point2f>;

  struct DetectorParams {
    int maxCorners{};
    double qualityLevel{};
    double minDistance{};
    cv::Mat mask;
  };

  FeatureDetector(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  FeatureDetector()
      : FeatureDetector(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~FeatureDetector() {}

 private:
  void loadParams();
  void detectFeatures(cv_bridge::CvImageConstPtr imagePtr, const DetectorParams &params, Features &features);
  void drawFeatures(cv_bridge::CvImagePtr imagePtr, const Features &features);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber imageSub_;
  ros::Publisher imagePub_;
  int waitTimeSecond_{30};

  Features features_;
  DetectorParams params_;
};

FeatureDetector::FeatureDetector(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  imageSub_ =
      nh_.subscribe("image_topic", 1, &FeatureDetector::imageCallback, this);

  imagePub_ = nh_.advertise<sensor_msgs::Image>("feature_image", 1000);

  loadParams();
}

void FeatureDetector::loadParams() {
  pnh_.getParam("waitTimeSecond", waitTimeSecond_);

  pnh_.getParam("maxCorners", params_.maxCorners);
  pnh_.getParam("minDistance", params_.minDistance);
  pnh_.getParam("qualityLevel", params_.qualityLevel);
}

void FeatureDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr ptrMono8;
  cv_bridge::CvImagePtr ptrRBG8;
  ptrMono8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  ptrRBG8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

  detectFeatures(ptrMono8, params_, features_);
  drawFeatures(ptrRBG8, features_);

  imagePub_.publish(ptrRBG8->toImageMsg());

  // cv::imshow("view", ptrRBG8->image);
  // cv::waitKey(waitTimeSecond_);
}

void FeatureDetector::detectFeatures(cv_bridge::CvImageConstPtr imagePtr, const DetectorParams &params, Features &features) {
  cv::goodFeaturesToTrack(imagePtr->image, features, params.maxCorners,
                          params.qualityLevel, params.minDistance,
                          params.mask);
}

void FeatureDetector::drawFeatures(cv_bridge::CvImagePtr imagePtr, const Features &features) {
  for (const auto &ftr : features) {
    cv::circle(imagePtr->image, ftr, 2, cv::Scalar(0, 255, 0), 2);
  }
}

}  // namespace lec4

int main(int argc, char** argv) {
  ros::init(argc, argv, "exp4_image_subscriber");
  std::cout << "===node exp4_image_subscriber starts===" << std::endl;

  lec4::FeatureDetector sub;

  ros::spin();
  return 0;
}