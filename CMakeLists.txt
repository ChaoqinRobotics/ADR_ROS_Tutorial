cmake_minimum_required(VERSION 3.0.2)
project(adr_ros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  nav_msgs
  roscpp
  rospy
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(

)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(exp0_basic_node
  src/exp0_basic_node.cpp
)
target_link_libraries(exp0_basic_node
  ${catkin_LIBRARIES})

add_executable(exp1_image_subscriber
  src/exp1_image_subscriber.cpp
)
target_link_libraries(exp1_image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(exp2_image_subscriber
  src/exp2_image_subscriber.cpp
)
target_link_libraries(exp2_image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(exp3_image_subscriber
  src/exp3_image_subscriber.cpp
)
target_link_libraries(exp3_image_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(exp4_feature_detector
  src/exp4_feature_detector.cpp
  src/feature_detector.cpp
)
target_link_libraries(exp4_feature_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})