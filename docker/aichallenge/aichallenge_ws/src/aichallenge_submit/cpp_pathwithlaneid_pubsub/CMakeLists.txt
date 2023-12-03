cmake_minimum_required(VERSION 3.14)
project(cpp_pathwithlaneid_pubsub)
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(cpp_pathwithlaneid_pubsub
  src/cpp_pathwithlaneid_pubsub.cpp
  src/cpp_pathwithlaneid_pubsub_impl.cpp
)

ament_auto_package(INSTALL_TO_SHARE)

