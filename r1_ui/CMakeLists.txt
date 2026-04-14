cmake_minimum_required(VERSION 3.8)
project(r1_ui)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(r1_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_executable(r1_controller_gateway_node src/r1_controller_gateway_node.cpp)
ament_target_dependencies(r1_controller_gateway_node
  r1_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS
  r1_controller_gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/r1_aruco_display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
