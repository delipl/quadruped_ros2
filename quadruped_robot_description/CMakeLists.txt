cmake_minimum_required(VERSION 3.8)
project(quadruped_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)


install(DIRECTORY launch meshes urdf rviz
        DESTINATION share/${PROJECT_NAME})


ament_environment_hooks(
          "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/setup_envs.sh.in")
ament_package()
