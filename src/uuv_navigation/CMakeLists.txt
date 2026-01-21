cmake_minimum_required(VERSION 3.8)
project(uuv_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Directorios del paquete
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}
)

# Nodos del paquete
install(PROGRAMS scripts/bezier_trayectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bezier_trayectory
)

install(PROGRAMS scripts/line_trayectory.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_trayectory
)

install(PROGRAMS scripts/los.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME los
)

install(PROGRAMS scripts/mapeador.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mapeador
)

add_executable(bezier src/bezier.cpp)

target_include_directories(bezier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)

ament_target_dependencies(bezier rclcpp nav_msgs visualization_msgs std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  bezier
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
