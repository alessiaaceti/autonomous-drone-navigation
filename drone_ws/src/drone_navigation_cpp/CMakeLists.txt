cmake_minimum_required(VERSION 3.8)
project(drone_navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(position_listener src/position_listener.cpp)

ament_target_dependencies(
  position_listener
  rclcpp
  px4_msgs
)

install(TARGETS
  position_listener
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(offboard_takeoff src/offboard_takeoff.cpp)

ament_target_dependencies(
  offboard_takeoff
  rclcpp
  px4_msgs
)

install(TARGETS
  offboard_takeoff
  DESTINATION lib/${PROJECT_NAME}
)

# Nodo Offboard Path
add_executable(offboard_path src/offboard_path.cpp)
ament_target_dependencies(
  offboard_path
  rclcpp
  px4_msgs
)
install(TARGETS
  offboard_path
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
