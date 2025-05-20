cmake_minimum_required(VERSION 3.10)
project(quadbar_kinematics)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(kinematics_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)

# Library
add_library(${PROJECT_NAME} SHARED
  src/quadbar_kinematics.cpp
  src/leg.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  kinematics_interface
  pluginlib
  Eigen3
)

target_link_libraries(${PROJECT_NAME}  Eigen3::Eigen)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Register as a plugin
pluginlib_export_plugin_description_file(quadbar_kinematics plugin.xml)

# Install targets
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Install plugin XML
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_export_dependencies(rclcpp quadbar_kinematics pluginlib Eigen3)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()
