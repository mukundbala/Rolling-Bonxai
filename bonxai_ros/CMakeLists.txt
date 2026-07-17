cmake_minimum_required(VERSION 3.8)
project(bonxai_ros)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

# Bonxai dependencies
find_package(bonxai_core REQUIRED)
find_package(bonxai_map REQUIRED)
find_package(bonxai_msgs REQUIRED)

# Create BonxaiServer library (for component and linking)
add_library(bonxai_server_lib SHARED
  src/bonxai_server.cpp
)

# Set C++17 standard
target_compile_features(bonxai_server_lib PUBLIC cxx_std_17)

# Include directories
target_include_directories(bonxai_server_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# Link with Bonxai libraries
target_link_libraries(bonxai_server_lib
  PUBLIC
    bonxai_core::bonxai_core
    bonxai_map::bonxai_map
    ${PCL_LIBRARIES}
)

# Set ROS2 dependencies
ament_target_dependencies(bonxai_server_lib
  PUBLIC
    rclcpp
    rclcpp_components
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_eigen
    pcl_conversions
    bonxai_msgs
)

# Register as composable node (for component containers)
rclcpp_components_register_node(bonxai_server_lib
  PLUGIN "Bonxai::BonxaiServer"
  EXECUTABLE bonxai_server_component
)

# Create standalone executable from main.cpp
add_executable(bonxai_server_node
  src/main.cpp
)

# Link standalone executable to the library
target_link_libraries(bonxai_server_node
  bonxai_server_lib
)

# Install library
install(
  TARGETS bonxai_server_lib
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install standalone executable
install(
  TARGETS bonxai_server_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install params files
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Export targets
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

# Export dependencies
ament_export_dependencies(
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
  PCL
  Eigen3
  bonxai_core
  bonxai_map
  bonxai_msgs
)

# Export include directories
ament_export_include_directories(include)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
