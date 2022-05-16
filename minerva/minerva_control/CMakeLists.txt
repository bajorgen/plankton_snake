cmake_minimum_required(VERSION 3.5)
project(minerva_control)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "srv/GetThrusterCurve.srv"
  "srv/ThrusterManagerInfo.srv" 
  "srv/SetThrusterManagerConfig.srv"
  "srv/GetThrusterManagerConfig.srv"
  DEPENDENCIES 
  builtin_interfaces 
  std_msgs
)

# rosidl_target_interfaces(publish_address_book
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")


ament_python_install_package(uuv_thrusters PACKAGE_DIR src/uuv_thrusters)
ament_python_install_package(uuv_thrusters.models PACKAGE_DIR src/uuv_thrusters/models)
ament_python_install_package(PID PACKAGE_DIR src/PID)

install(
  PROGRAMS scripts/thruster_allocator.py
           scripts/position_control.py
	   scripts/acceleration_control.py
	   scripts/velocity_control.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*~" EXCLUDE)


ament_package()
