cmake_minimum_required(VERSION 3.14)
project(my_cpp_lqr)
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
autoware_package()
ament_auto_find_build_dependencies()

ament_auto_add_executable(my_cpp_lqr
  src/my_cpp_lqr.cpp
)

ament_auto_package(INSTALL_TO_SHARE)

