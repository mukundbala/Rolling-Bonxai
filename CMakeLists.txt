cmake_minimum_required(VERSION 3.8)
project(rolling-map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(message_filters REQUIRED)

# ====================================================
# Header-only Library: bonxai_core
add_library(bonxai_core INTERFACE)

target_include_directories(bonxai_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(bonxai_core INTERFACE Eigen3::Eigen)

# ====================================================


# ====================================================
# Compiled Library: bonxai_map
add_library(bonxai_map
  src/probabilistic_map_old.cpp
)

target_include_directories(bonxai_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(bonxai_map PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(bonxai_map
  bonxai_core
  ${PCL_LIBRARIES}
)

# ====================================================

# ====================================================
include_directories(
  include/rolling_map
)

add_executable(rolling_map_node
  # src/map_handler.cpp
  # src/chunk_cache.cpp
  # src/chunk_manager.cpp
  src/rolling_map.cpp
  src/rolling_map_node.cpp
)

target_include_directories(rolling_map_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  include/rolling_map
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_compile_definitions(rolling_map_node PRIVATE ${PCL_DEFINITIONS})

target_link_libraries(rolling_map_node
  bonxai_map
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(rolling_map_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  PCL
  Eigen3
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  angles
  pcl_conversions
)


# ====================================================
# Install
install(DIRECTORY include/ DESTINATION include)

install(TARGETS bonxai_core bonxai_map
  EXPORT bonxai_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS rolling_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY configs
  DESTINATION share/${PROJECT_NAME}
)

# ====================================================
# ament exports
ament_export_targets(bonxai_targets)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  lifecycle_msgs
  tf2
  tf2_ros
  PCL
  Eigen3
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  angles
)


ament_package()
