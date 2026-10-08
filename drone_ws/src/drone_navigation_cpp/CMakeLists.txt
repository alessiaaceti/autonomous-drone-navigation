cmake_minimum_required(VERSION 3.8)
project(drone_navigation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(OpenCV REQUIRED)


# ------------------------------------------------------------
# Custom Messages
# ------------------------------------------------------------

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetState.msg"
  "msg/DecisionState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ------------------------------------------------------------
# Position Listener
# ------------------------------------------------------------

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

# ------------------------------------------------------------
# Offboard Takeoff
# ------------------------------------------------------------

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

# ------------------------------------------------------------
# Offboard Path
# ------------------------------------------------------------

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

# ------------------------------------------------------------
# Camera Viewer / Vision Node
# ------------------------------------------------------------

add_executable(camera_viewer src/camera_viewer.cpp)
ament_target_dependencies(
  camera_viewer
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(camera_viewer ${OpenCV_LIBS})
install(TARGETS camera_viewer DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------
# Depth-based Obstacle Detector
# ------------------------------------------------------------

add_executable(obstacle_detector src/obstacle_detector.cpp)
ament_target_dependencies(
  obstacle_detector
  rclcpp
  sensor_msgs
  cv_bridge
)
install(TARGETS obstacle_detector DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------
# Obstacle Avoidance Decision Node
# ------------------------------------------------------------

add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(
  obstacle_avoidance
  rclcpp
  std_msgs
)
install(TARGETS obstacle_avoidance DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------
# Offboard Avoidance Controller
# ------------------------------------------------------------

add_executable(offboard_avoidance src/offboard_avoidance.cpp)

ament_target_dependencies(
  offboard_avoidance
  rclcpp
  std_msgs
  px4_msgs
)

install(
  TARGETS offboard_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Depth Visualization
# ------------------------------------------------------------

add_executable(depth_visualizer src/depth_visualizer.cpp)
ament_target_dependencies(
  depth_visualizer
  rclcpp
  sensor_msgs
  cv_bridge
)
target_link_libraries(depth_visualizer ${OpenCV_LIBS})
install(TARGETS depth_visualizer DESTINATION lib/${PROJECT_NAME})

# ------------------------------------------------------------
# Drone Controller
# ------------------------------------------------------------

add_executable(drone_controller src/drone_controller.cpp)

ament_target_dependencies(
  drone_controller
  rclcpp
  std_msgs
  px4_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  drone_controller
  "${cpp_typesupport_target}"
)

install(TARGETS
  drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------
# YOLO Target Tracker
# ------------------------------------------------------------

add_executable(yolo_target_tracker src/yolo_target_tracker.cpp)

ament_target_dependencies(
  yolo_target_tracker
  rclcpp
  vision_msgs
)

rosidl_get_typesupport_target(
  yolo_tracker_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  yolo_target_tracker
  "${yolo_tracker_typesupport_target}"
)

install(
  TARGETS yolo_target_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Decision Layer
# ------------------------------------------------------------

add_executable(decision_layer src/decision_layer.cpp)

ament_target_dependencies(
  decision_layer
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(
  decision_layer_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  decision_layer
  "${decision_layer_typesupport_target}"
)

install(
  TARGETS decision_layer
  DESTINATION lib/${PROJECT_NAME}
)

# ------------------------------------------------------------
# Testing
# ------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
