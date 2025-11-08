cmake_minimum_required(VERSION 3.8)
project(r1_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sabacan_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r1_msgs REQUIRED)

add_executable(r1_mecanum_node src/r1_mecanum_node.cpp)
add_executable(r1_sabacan_msgs_converter_node src/r1_sabacan_msgs_converter_node.cpp)
add_executable(r1_odometry_node src/r1_odometry_node.cpp)

ament_target_dependencies(r1_mecanum_node
  geometry_msgs
  rclcpp
  std_msgs
  r1_msgs
)

ament_target_dependencies(r1_sabacan_msgs_converter_node
  rclcpp
  sabacan_msgs
  std_msgs
  r1_msgs
)

ament_target_dependencies(r1_odometry_node
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  r1_msgs
)

# インクルードディレクトリを設定
target_include_directories(r1_mecanum_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(r1_sabacan_msgs_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(r1_odometry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS r1_mecanum_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS r1_sabacan_msgs_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS r1_odometry_node
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
