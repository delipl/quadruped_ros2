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
    example_interfaces)

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

# ============================================================
# 1. BIBLIOTEKA INTERFEJSU (dla pluginów)
# ============================================================
add_library(quadruped_controller_interfaces INTERFACE)
target_include_directories(
  quadruped_controller_interfaces
  INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  quadruped_controller_interfaces
  INTERFACE
  rclcpp
  sensor_msgs
  geometry_msgs
  pluginlib
  Eigen3)

# ============================================================
# 1. BIBLIOTEKA LEG PLUGIN (klasa bazowa)
# ============================================================
add_library(quadruped_controller_leg_plugin SHARED src/leg_plugin.cpp)

target_include_directories(
  quadruped_controller_leg_plugin
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}> ${EIGEN3_INCLUDE_DIRS})

ament_target_dependencies(quadruped_controller_leg_plugin rclcpp sensor_msgs
                          geometry_msgs pluginlib Eigen3)

target_link_libraries(quadruped_controller_leg_plugin
                      quadruped_controller_interfaces ${EIGEN3_LIBRARIES})

target_compile_definitions(
  quadruped_controller_leg_plugin PRIVATE PLUGINLIB__DISABLE_BOOST_FUNCTIONS
                                          QUADRUPED_CONTROLLER_BUILDING_DLL)

# ============================================================
# 1. PARAMETRY (INTERFACE)
# ============================================================
generate_parameter_library(
  quadruped_controller_parameters src/quadruped_controller.yaml
  include/quadruped_controller/validate_quadruped_controller_parameters.hpp)

# ============================================================
# 1. GŁÓWNY KONTROLER
# ============================================================
add_library(quadruped_controller SHARED src/quadruped_controller.cpp)

target_include_directories(
  quadruped_controller
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>" ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(quadruped_controller quadruped_controller_parameters
                      quadruped_controller_interfaces ${EIGEN3_LIBRARIES})

ament_target_dependencies(quadruped_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(quadruped_controller
                           PRIVATE "QUADRUPED_CONTROLLER_BUILDING_DLL")

# ============================================================
# 1. REJESTRACJA PLUGINU KONTROLERA
# ============================================================
pluginlib_export_plugin_description_file(controller_interface
                                         quadruped_controller.xml)

# ============================================================
# 1. INSTALACJA
# ============================================================
# Instalacja bibliotek (SHARED + biblioteki INTERFACE — muszą być w tym samym
# export set, bo są linkowane PUBLIC do quadruped_controller / leg_plugin)
install(
  TARGETS quadruped_controller quadruped_controller_leg_plugin
          quadruped_controller_interfaces quadruped_controller_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
# ============================================================
# 1. PYTHON
# ============================================================
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/twist_to_trajectory.py DESTINATION lib/${PROJECT_NAME})

# ============================================================
# 1. EKSPORT
# ============================================================
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Eigen3 pluginlib
                          generate_parameter_library)
ament_export_libraries(quadruped_controller)
ament_export_libraries(quadruped_controller_leg_plugin)

ament_package()
