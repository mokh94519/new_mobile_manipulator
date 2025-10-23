cmake_minimum_required(VERSION 3.16)
project(mobile_manipulator_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config urdf worlds models resource
  DESTINATION share/${PROJECT_NAME})

ament_python_install_package(mobile_manipulator_bringup)

install(PROGRAMS 
  mobile_manipulator_bringup/explore_and_collect.py
  mobile_manipulator_bringup/object_detector.py
  mobile_manipulator_bringup/pick_place.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
