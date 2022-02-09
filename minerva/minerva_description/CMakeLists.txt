cmake_minimum_required(VERSION 3.5)
project(minerva_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes robots urdf
	DESTINATION share/${PROJECT_NAME}
	PATTERN "*~"
	EXCLUDE)

  

ament_package()
