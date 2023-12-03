cmake_minimum_required(VERSION 3.14)
project(path_subscriber)

find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(path_subscriber
  src/path_subscriber.cpp
)

ament_auto_package(INSTALL_TO_SHARE)

