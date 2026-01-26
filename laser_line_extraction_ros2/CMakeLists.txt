cmake_minimum_required(VERSION 3.8)
project(laser_line_extraction_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # 1. Added std_msgs
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LineSegment.msg"
  "msg/LineSegmentList.msg"
  DEPENDENCIES sensor_msgs visualization_msgs geometry_msgs std_msgs
)

add_library(line_extraction_lib
  src/line.cpp
  src/line_extraction.cpp
  src/line_extraction_ros.cpp
)

target_include_directories(line_extraction_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 2. Link library to typesupport so it can find generated headers
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(line_extraction_lib 
  Eigen3::Eigen 
  "${cpp_typesupport_target}"
)

# 3. Force header generation before library compilation
add_dependencies(line_extraction_lib ${PROJECT_NAME}__rosidl_typesupport_cpp)

ament_target_dependencies(line_extraction_lib 
  rclcpp 
  std_msgs 
  sensor_msgs 
  visualization_msgs 
  geometry_msgs
)

add_executable(line_extraction_node src/line_extraction_node.cpp)
target_link_libraries(line_extraction_node 
  line_extraction_lib 
  "${cpp_typesupport_target}"
)

install(TARGETS line_extraction_node line_extraction_lib
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()