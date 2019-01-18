cmake_minimum_required(VERSION 2.8.3)
project(testit_sut)

find_package(catkin REQUIRED COMPONENTS
  rospy
  testit_msgs
)

catkin_python_setup()

catkin_package(
)
