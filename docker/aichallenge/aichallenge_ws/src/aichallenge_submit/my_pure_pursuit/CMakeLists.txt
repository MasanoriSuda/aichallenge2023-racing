cmake_minimum_required(VERSION 3.14)
project(my_pure_pursuit)
find_package(ament_cmake REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
autoware_package()

ament_auto_add_executable(my_pure_pursuit
  src/my_pure_pursuit.cpp
  src/my_pure_pursuit_impl.cpp
)

ament_auto_package(INSTALL_TO_SHARE)


