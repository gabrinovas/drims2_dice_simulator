cmake_minimum_required(VERSION 3.8)
project(drims2_dice_simulator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_py REQUIRED)

# Find custom package
find_package(drims2_msgs REQUIRED)

# Instalar el paquete Python
ament_python_install_package(${PROJECT_NAME})

# Instalar el script como ejecutable
install(PROGRAMS
  scripts/dice_spawner.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install urdf meshes
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()