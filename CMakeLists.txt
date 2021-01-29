cmake_minimum_required(VERSION 3.0.2)
project(jaguar_utils)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  PCL
  pcl_ros
  tf_conversions
  eigen_conversions
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_extractor
  CATKIN_DEPENDS roscpp rospy tf tf_conversions eigen_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(odom_base_publisher src/odom_tf.cpp)
target_link_libraries(odom_base_publisher ${catkin_LIBRARIES})

add_executable(odom_base_listener src/odom_listener.cpp)
target_link_libraries(odom_base_listener ${catkin_LIBRARIES})

add_executable(pcd_reader src/pcd_reader.cpp)
target_link_libraries(pcd_reader ${catkin_LIBRARIES})

add_executable(pcl_counter src/pcl_count.cpp)
target_link_libraries(pcl_counter ${catkin_LIBRARIES})

add_executable(scale_odom src/scale_twist.cpp)
target_link_libraries(scale_odom ${catkin_LIBRARIES})
