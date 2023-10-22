#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>

namespace lec2 {

class ImageSubscriber {
 public:
  ImageSubscriber(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ImageSubscriber()
      : ImageSubscriber(ros::NodeHandle(), ros::NodeHandle("~")) {}
  ~ImageSubscriber() {}

 private:
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber imageSub_;
  int waitTimeMilliseconds_{30};
};

ImageSubscriber::ImageSubscriber(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  imageSub_ =
      nh_.subscribe("image_topic", 1, &ImageSubscriber::imageCallback, this);

  pnh_.getParam("waitTimeMilliseconds", waitTimeMilliseconds_);
}

void ImageSubscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::imshow("view", cv_bridge::toCvShare(msg, "rgb8")->image);
  cv::waitKey(waitTimeMilliseconds_);
}

}  // namespace lec2

int main(int argc, char** argv) {
  ros::init(argc, argv, "exp2_image_subscriber");
  std::cout << "===node exp2_image_subscriber starts===" << std::endl;

  lec2::ImageSubscriber sub;

  ros::spin();
  return 0;
}

