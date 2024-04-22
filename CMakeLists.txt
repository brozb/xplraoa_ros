cmake_minimum_required(VERSION 3.0.2)
project(xplraoa_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  Angles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES xplraoa_ros
#  CATKIN_DEPENDS geometry_msgs ros_numpy rospy std_msgs tf2 tf2_py tf2_ros
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS
    scripts/bt_angle.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
