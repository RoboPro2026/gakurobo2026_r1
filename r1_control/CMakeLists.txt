cmake_minimum_required(VERSION 3.8)
project(r1_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # r1_controlは重い処理が多いので、-O3でコンパイルする
  add_compile_options(-std=c++20 -Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development MODULE REQUIRED)
find_package(pybind11 REQUIRED)

add_executable(test_minimum_jerk_node example/test_minimum_jerk_node.cpp)
add_executable(test_spline_node example/test_spline_node.cpp)
add_executable(test_trajectory_planner_node example/test_trajectory_planner_node.cpp)
add_executable(test_trapezoid_node example/test_trapezoid_node.cpp)
add_executable(r1_chassis_control_node src/r1_chassis_control_node.cpp)
add_executable(r1_dummy_odometry_node src/r1_dummy_odometry_node.cpp)

pybind11_add_module(trajectory_planner src/trajectory_planner.cpp)

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

ament_target_dependencies(r1_chassis_control_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

ament_target_dependencies(r1_dummy_odometry_node
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)


target_include_directories(test_minimum_jerk_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(test_spline_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(test_trajectory_planner_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(test_trapezoid_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(trajectory_planner PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(r1_chassis_control_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(r1_dummy_odometry_node PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(trajectory_planner
  PRIVATE
    rclcpp::rclcpp
)

install(TARGETS test_minimum_jerk_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_spline_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS test_trapezoid_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS r1_chassis_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS r1_dummy_odometry_node
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
