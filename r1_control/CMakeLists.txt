cmake_minimum_required(VERSION 3.8)
project(r1_control)

# r1_control は計算量が比較的大きいため、この package では build type を
# 明示的に Release に固定します。最適化レベルは add_compile_options() で個別に
# 指定せず、CMake の Release 設定に任せます。
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
find_package(magic_enum CONFIG REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(r1_msgs REQUIRED)
find_package(r1_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development MODULE REQUIRED)
find_package(pybind11 REQUIRED)

set(COMMON_INCLUDE_DIRS
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(r1_chassis_control_node src/r1_chassis_control_node.cpp)
add_executable(r1_dummy_map_node src/r1_dummy_map_node.cpp)
add_executable(r1_dummy_odometry_node src/r1_dummy_odometry_node.cpp)
add_executable(r1_laser_filter_node src/r1_laser_filter_node.cpp)
add_executable(test_minimum_jerk_node example/test_minimum_jerk_node.cpp)
add_executable(test_spline_node example/test_spline_node.cpp)
add_executable(test_trajectory_planner_node example/test_trajectory_planner_node.cpp)
add_executable(test_trapezoid_node example/test_trapezoid_node.cpp)

pybind11_add_module(trajectory_planner src/trajectory_planner.cpp)

ament_target_dependencies(r1_chassis_control_node
  geometry_msgs
  nav_msgs
  r1_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

ament_target_dependencies(r1_dummy_map_node
  geometry_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(r1_dummy_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(r1_laser_filter_node
  rclcpp
  r1_util
  sensor_msgs
)

ament_target_dependencies(test_minimum_jerk_node
  rclcpp
)

ament_target_dependencies(test_spline_node
  rclcpp
)

ament_target_dependencies(test_trajectory_planner_node
  rclcpp
)

ament_target_dependencies(test_trapezoid_node
  rclcpp
)

target_link_libraries(r1_chassis_control_node r1_util::r1_util magic_enum::magic_enum)
target_link_libraries(r1_dummy_odometry_node r1_util::r1_util)
target_link_libraries(r1_laser_filter_node r1_util::r1_util)
target_link_libraries(test_minimum_jerk_node r1_util::r1_util)
target_link_libraries(test_spline_node r1_util::r1_util)
target_link_libraries(test_trajectory_planner_node r1_util::r1_util)
target_link_libraries(test_trapezoid_node r1_util::r1_util)

target_link_libraries(trajectory_planner
  PRIVATE
    rclcpp::rclcpp
    r1_util::r1_util
)

target_include_directories(r1_chassis_control_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_dummy_map_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_dummy_odometry_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(r1_laser_filter_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(test_minimum_jerk_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(test_spline_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(test_trajectory_planner_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(test_trapezoid_node PUBLIC ${COMMON_INCLUDE_DIRS})
target_include_directories(trajectory_planner PUBLIC ${COMMON_INCLUDE_DIRS})

install(TARGETS
  r1_chassis_control_node
  r1_dummy_map_node
  r1_dummy_odometry_node
  r1_laser_filter_node
  test_minimum_jerk_node
  test_spline_node
  test_trajectory_planner_node
  test_trapezoid_node
  DESTINATION lib/${PROJECT_NAME}
)

# 例 lib/python3.10/site-packages
install(TARGETS trajectory_planner
  DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
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
