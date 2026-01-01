cmake_minimum_required(VERSION 3.8)
project(uuv_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 1. Buscar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Buscar dependencias de Ignition
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-transport11 REQUIRED)

# 3. Incluir directorios (Aquí está el truco: usamos las variables de INCLUDE_DIRS)
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${ignition-gazebo6_INCLUDE_DIRS}
  ${ignition-plugin1_INCLUDE_DIRS}
  ${ignition-transport11_INCLUDE_DIRS}
)

# 4. Crear la librería del plugin
add_library(SubmarineDynamics SHARED
  src/SubmarineDynamics.cpp
  src/uuv_dynamic_model.cpp
)

# 5. Vincular librerías
target_link_libraries(SubmarineDynamics
  ${ignition-gazebo6_LIBRARIES}
  ignition-plugin1::all
  ignition-transport11::core
  Eigen3::Eigen
)

# 6. Vincular dependencias de ROS 2
ament_target_dependencies(SubmarineDynamics
  rclcpp
  std_msgs
  geometry_msgs
)

# 7. Instalación
install(TARGETS SubmarineDynamics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()