cmake_minimum_required(VERSION 3.8)
project(r1_machine)

# r1_machine は実行周期に対する負荷が高いため、この package では build type を
# 明示的に Release に固定します。最適化レベルは CMake の Release 設定に任せます。
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)

# ccache が入っていれば、コンパイル結果を再利用して再ビルド時間を短縮します。
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sabacan_msgs REQUIRED)
find_package(sabacan_single_control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r1_msgs REQUIRED)
find_package(r1_util REQUIRED)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(r1_angle_motion_node src/r1_angle_motion_node.cpp)
add_executable(r1_linear_motion_node src/r1_linear_motion_node.cpp)
add_executable(r1_machine_manage_node src/r1_machine_manage_node.cpp)
add_executable(r1_mecanum_node src/r1_mecanum_node.cpp)
add_executable(r1_odometry_node src/r1_odometry_node.cpp)
add_executable(r1_swerve_drive_node src/r1_swerve_drive_node.cpp)

ament_target_dependencies(r1_angle_motion_node
  rclcpp
  std_msgs
  r1_msgs
  r1_util
)

ament_target_dependencies(r1_linear_motion_node
  rclcpp
  std_msgs
  r1_msgs
  r1_util
)

ament_target_dependencies(r1_machine_manage_node
  rclcpp
  sabacan_msgs
  sabacan_single_control_msgs
  std_msgs
  r1_msgs
  r1_util
)

ament_target_dependencies(r1_mecanum_node
  geometry_msgs
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  r1_msgs
  r1_util
)

ament_target_dependencies(r1_odometry_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2
  tf2_ros
  nav_msgs
  sensor_msgs
  r1_msgs
  r1_util
)

ament_target_dependencies(r1_swerve_drive_node
  geometry_msgs
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  r1_msgs
  r1_util
)

target_link_libraries(r1_angle_motion_node r1_util::r1_util)
target_link_libraries(r1_linear_motion_node r1_util::r1_util)
target_link_libraries(r1_machine_manage_node r1_util::r1_util)
target_link_libraries(r1_mecanum_node r1_util::r1_util)
target_link_libraries(r1_odometry_node r1_util::r1_util)
target_link_libraries(r1_swerve_drive_node r1_util::r1_util)

target_include_directories(r1_angle_motion_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_linear_motion_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_machine_manage_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_mecanum_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_odometry_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_swerve_drive_node PUBLIC ${COMMON_INCLUDE_DIRS})

install(TARGETS
  r1_angle_motion_node
  r1_linear_motion_node
  r1_machine_manage_node
  r1_mecanum_node
  r1_odometry_node
  r1_swerve_drive_node
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
