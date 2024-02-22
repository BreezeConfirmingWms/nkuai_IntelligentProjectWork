cmake_minimum_required(VERSION 3.0.2)
project(my_robot_gazebo)

find_package(gazebo_ros REQUIRED)

catkin_package(CATKIN_DEPENDS gazebo_ros)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
