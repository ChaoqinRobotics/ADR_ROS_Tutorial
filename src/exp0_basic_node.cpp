#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exp0_basic_node");

  ros::NodeHandle nh;

  std::cout << "===node starts===" << std::endl;

  ros::spin();

  std::cout << "===node ends===" << std::endl;
}