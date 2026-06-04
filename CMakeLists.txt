cmake_minimum_required(VERSION 3.5)
project(cone_fused)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -march=native -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mmr_base REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
)

# Build Node class
add_library(${PROJECT_NAME} 
  src/${PROJECT_NAME}.cpp
  src/ekf_odom.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  nav_msgs
  visualization_msgs
  mmr_base
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  mmr_base
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  Eigen3::Eigen
)

# Install target
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config directory
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
