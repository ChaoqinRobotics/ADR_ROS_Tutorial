#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud.h>

namespace lec3 {

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
  ros::Publisher imagePub_;
  ros::Publisher pclPub_;
  int waitTimeMilliseconds_{30};
};

ImageSubscriber::ImageSubscriber(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh) {
  imageSub_ =
      nh_.subscribe("image_topic", 1, &ImageSubscriber::imageCallback, this);

  imagePub_ = nh_.advertise<sensor_msgs::Image>("feature_image",1000);

  pclPub_ = nh_.advertise<sensor_msgs::PointCloud>("features", 1000);

  pnh_.getParam("waitTimeMilliseconds", waitTimeMilliseconds_);
}

void ImageSubscriber::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImageConstPtr ptrMono8, ptrRBG8;
  ptrMono8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  ptrRBG8 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);

  std::vector<cv::Point2f> corners;
  int maxCorners = 120;
  double qualityLevel = 0.01;
  double minDistance = 30.;
  cv::Mat mask;
  cv::goodFeaturesToTrack(ptrMono8->image, corners, maxCorners, qualityLevel, minDistance, mask);

  cv::Mat image = ptrRBG8->image;
  for (unsigned int j = 0; j < corners.size(); j++) {
    cv::circle(image, corners[j], 2, cv::Scalar(0, 255, 0), 2);
  }

  cv::imshow("view", image);
  cv::waitKey(waitTimeMilliseconds_);

  // Publish corner features;
  sensor_msgs::PointCloudPtr featurePoints(new sensor_msgs::PointCloud);
  featurePoints->header.stamp = ros::Time::now();
  featurePoints->header.frame_id = "image";

  for (int i = 0; i < corners.size(); i++) {
    geometry_msgs::Point32 ft;
    ft.x = corners[i].x;
    ft.y = corners[i].y;
    ft.z = 1;
    featurePoints->points.push_back(ft);
  }

  pclPub_.publish(featurePoints);
}

}  // namespace lec3

int main(int argc, char** argv) {
  ros::init(argc, argv, "exp3_image_subscriber");
  std::cout << "===node exp3_image_subscriber starts===" << std::endl;

  lec3::ImageSubscriber sub;

  ros::spin();
  return 0;
}