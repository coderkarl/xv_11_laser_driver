cmake_minimum_required(VERSION 3.5)
project(xv_11_laser_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(neato_laser_publisher 
	src/neato_laser_publisher.cpp
	src/xv11_laser.cpp
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()