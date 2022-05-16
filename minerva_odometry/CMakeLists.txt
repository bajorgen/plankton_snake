cmake_minimum_required(VERSION 3.5)
project(minerva_odometry)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)


ament_python_install_package(minerva/odometry PACKAGE_DIR src/odometry)

install(
  PROGRAMS scripts/odometry.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)


ament_package()
