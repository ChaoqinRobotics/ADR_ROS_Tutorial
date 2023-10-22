#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>

namespace lec4 {

class FeatureDetector {
 public:
  static constexpr double COLOR_RGB[3] = {0, 255, 0};
  static constexpr double CIRCLE_RADIUS = 2.0;
  static constexpr double CIRCLE_THICKNESS = 2.0;

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
  void detectFeatures(cv_bridge::CvImageConstPtr imagePtr,
                      const DetectorParams& params, Features& features);
  void drawFeatures(cv_bridge::CvImagePtr imagePtr, const Features& features);
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber imageSub_;
  ros::Publisher imagePub_;
  int waitTimeMilliseconds_{30};

  Features features_;
  DetectorParams params_;
};

}  // namespace lec4