cmake_minimum_required(VERSION 3.8)
project(r1_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development NumPy REQUIRED)

add_executable(trapezoid_designer src/trapezoid_designer.cpp)
add_executable(spline_test_node example/spline_test_node.cpp)
add_executable(trapezoid_test_node example/trapezoid_test_node.cpp)

ament_target_dependencies(trapezoid_designer
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(spline_test_node
  rclcpp
)

ament_target_dependencies(trapezoid_test_node
  rclcpp
)

target_include_directories(trapezoid_designer PUBLIC
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(spline_test_node PUBLIC
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(trapezoid_test_node PUBLIC
  ${Python3_INCLUDE_DIRS}
  ${Python3_NumPy_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS trapezoid_designer
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS spline_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS trapezoid_test_node
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
