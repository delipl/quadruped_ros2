cmake_minimum_required(VERSION 3.5)
project(quadruped_controller)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_srvs
  tf2_ros
  trajectory_msgs
  quadruped_msgs
  visualization_msgs
  kinematics_interface
  )

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(generate_parameter_library REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add quadruped_controller library related compile commands
generate_parameter_library(quadruped_controller_parameters
  src/quadruped_controller.yaml
  include/quadruped_controller/validate_quadruped_controller_parameters.hpp
)
add_library(
  quadruped_controller
  SHARED
  src/quadruped_controller.cpp
  src/leg.cpp
)
target_include_directories(quadruped_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(quadruped_controller quadruped_controller_parameters)
ament_target_dependencies(quadruped_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(quadruped_controller PRIVATE "QUADRUPED_CONTROLLER_BUILDING_DLL")

pluginlib_export_plugin_description_file(
  controller_interface quadruped_controller.xml)

install(
  TARGETS
  quadruped_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# TESTS
# ament_add_gmock(test_load_quadruped_controller test/test_load_quadruped_controller.cpp)
# target_include_directories(test_load_quadruped_controller PRIVATE include)
# ament_target_dependencies(
# test_load_quadruped_controller
# controller_manager
# hardware_interface
# ros2_control_test_assets
# )

# add_rostest_with_parameters_gmock(test_quadruped_controller test/test_quadruped_controller.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/quadruped_controller_params.yaml)
# target_include_directories(test_quadruped_controller PRIVATE include)
# target_link_libraries(test_quadruped_controller quadruped_controller)
# ament_target_dependencies(
# test_quadruped_controller
# controller_interface
# hardware_interface
# )

# add_rostest_with_parameters_gmock(test_quadruped_controller_preceeding test/test_quadruped_controller_preceeding.cpp ${CMAKE_CURRENT_SOURCE_DIR}/test/quadruped_controller_preceeding_params.yaml)
# target_include_directories(test_quadruped_controller_preceeding PRIVATE include)
# target_link_libraries(test_quadruped_controller_preceeding quadruped_controller)
# ament_target_dependencies(
# test_quadruped_controller_preceeding
# controller_interface
# hardware_interface
# )
add_executable(quadruped_controller_node src/inverse_kinematics_test.cpp src/leg.cpp)
target_include_directories(quadruped_controller_node PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(quadruped_controller_node
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
target_link_libraries(quadruped_controller_node Eigen3::Eigen)

add_executable(controller src/controller.cpp)
target_include_directories(controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(controller ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(passive_joint_state_broadcaster src/effector_joint_state_publisher.cpp src/leg.cpp)

target_include_directories(passive_joint_state_broadcaster PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(passive_joint_state_broadcaster ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(passive_joint_state_broadcaster Eigen3::Eigen)

add_executable(base_inverse_kinematics src/base_inverse_kinematics.cpp)

target_include_directories(base_inverse_kinematics PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(base_inverse_kinematics ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(base_inverse_kinematics Eigen3::Eigen)

install(TARGETS quadruped_controller_node controller passive_joint_state_broadcaster base_inverse_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_export_include_directories(
  include
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(
  quadruped_controller
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  scripts/twist_to_trajectory.py
  DESTINATION lib/${PROJECT_NAME}
)
 

ament_package()