cmake_minimum_required(VERSION 3.8)
project(uuv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

install(DIRECTORY libs DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/pid.py DESTINATION lib/${PROJECT_NAME} RENAME pid)

# === Library ===
add_library(uuv_model_lib
  libs/model/uuv_dynamic_model.cpp
)

add_library(uuv_asmc_lib
  libs/controller_asmc/6dof_asmc.cpp
)

target_include_directories(uuv_model_lib PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/model
)

# === Executable ===
add_executable(dynamic_model_uuv src/dynamic_model_uuv.cpp)

add_executable(asmc src/asmc.cpp)

target_include_directories(dynamic_model_uuv PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/model
)

ament_target_dependencies(dynamic_model_uuv
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav2_msgs
  tf2_ros
)

ament_target_dependencies(asmc
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  nav2_msgs
  tf2_ros
)

target_link_libraries(dynamic_model_uuv
  uuv_model_lib
)

target_link_libraries(asmc
  uuv_asmc_lib
)

install(TARGETS dynamic_model_uuv DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/pid.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pid
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
