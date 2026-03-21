cmake_minimum_required(VERSION 3.8)
project(r1_main)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
# for C++ enum
find_package(magic_enum CONFIG REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r1_msgs REQUIRED)
find_package(r1_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sabacan_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(r1_main_node src/r1_main_node.cpp)
add_executable(test_node src/test_node.cpp)

ament_target_dependencies(r1_main_node
  geometry_msgs
  nav_msgs
  rclcpp
  sabacan_msgs
  sensor_msgs
  std_msgs
  r1_msgs
  r1_util
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(test_node
  rclcpp
  r1_msgs
  r1_util
  sabacan_msgs
  sensor_msgs
  std_msgs
)

target_link_libraries(r1_main_node magic_enum::magic_enum)
target_link_libraries(r1_main_node r1_util::r1_util)

# インクルードディレクトリを設定
target_include_directories(r1_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS r1_main_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_node
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
