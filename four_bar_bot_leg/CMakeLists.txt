cmake_minimum_required(VERSION 3.8)
project(four_bar_bot_leg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ============================================
# ZNAJDŹ WSZYSTKIE ZALEŻNOŚCI
# ============================================
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# WAŻNE: Znajdź Eigen3
find_package(Eigen3 REQUIRED) # <-- TO JEST KLUCZOWE

# Znajdź quadruped_controller
find_package(quadruped_controller REQUIRED)

# ============================================
# BIBLIOTEKA PLUGINU
# ============================================
add_library(four_bar_bot_leg SHARED src/four_bar_leg.cpp)

target_include_directories(
  four_bar_bot_leg
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIRS} # <-- DODAJ EIGEN
                                                             # INCLUDE
)

target_compile_definitions(four_bar_bot_leg
                           PRIVATE PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# Zależności
ament_target_dependencies(
  four_bar_bot_leg
  rclcpp
  sensor_msgs
  geometry_msgs
  pluginlib
  quadruped_controller
  Eigen3 # <-- DODAJ EIGEN3 DO DEPENDENCIES
)

target_link_libraries(
  four_bar_bot_leg ${EIGEN3_LIBRARIES} # <-- DODAJ LINKOWANIE EIGEN
)

# ============================================
# REJESTRACJA PLUGINU
# ============================================
pluginlib_export_plugin_description_file(four_bar_bot_leg
                                         four_bar_leg_plugin.xml)

# ============================================
# INSTALACJA
# ============================================
install(
  TARGETS four_bar_bot_leg
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(FILES four_bar_leg_plugin.xml DESTINATION share/${PROJECT_NAME})

# ============================================
# TESTY
# ============================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================
# EKSPORT
# ============================================
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp sensor_msgs geometry_msgs pluginlib quadruped_controller
  Eigen3 # <-- DODAJ EIGEN3 DO EKSPORTU
)
ament_export_include_directories(include)

ament_package()
