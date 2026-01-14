cmake_minimum_required(VERSION 3.8)
project(bno086)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(bno086_node src/bno086_node.cpp)
# example code
add_executable(bno086_subscription_node example/bno086_subscription_node.cpp)
add_executable(npm_uart_test_node example/npm_uart_test_node.cpp)
add_executable(npm_uart_can_node example/npm_uart_can_node.cpp)

ament_target_dependencies(bno086_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(bno086_subscription_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)

ament_target_dependencies(npm_uart_test_node
  rclcpp
)

ament_target_dependencies(npm_uart_can_node
  rclcpp
)


# インクルードディレクトリを設定
target_include_directories(bno086_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(bno086_subscription_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(npm_uart_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(npm_uart_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS bno086_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS bno086_subscription_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS npm_uart_test_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS npm_uart_can_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_bno086_driver test/test_bno086_driver.cpp)

  ament_target_dependencies(test_bno086_driver
    rclcpp
    ament_cmake_gtest
  )

  target_include_directories(test_bno086_driver PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

endif()

ament_package()
