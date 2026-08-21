cmake_minimum_required(VERSION 3.20)
project(lane_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include(FetchContent)
# Fetch nlohmann/json
FetchContent_Declare(
  json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG v3.12.0
)
FetchContent_MakeAvailable(json)
# Fetch SimpleBLE
FetchContent_Declare(
  simpleble
  GIT_REPOSITORY https://github.com/simpleble/simpleble
  GIT_TAG v0.10.3
  GIT_SHALLOW YES
  )
# Note that here we manually do what FetchContent_MakeAvailable() would do,
# except to ensure that the dependency can also get what it needs, we add
# custom logic between the FetchContent_Populate() and add_subdirectory()
# calls.
FetchContent_GetProperties(simpleble)
if(NOT simpleble_POPULATED)
    FetchContent_Populate(simpleble)
    list(APPEND CMAKE_MODULE_PATH "${simpleble_SOURCE_DIR}/cmake/find")
    add_subdirectory("${simpleble_SOURCE_DIR}/simpleble" "${simpleble_BINARY_DIR}")
endif()

# Suppress SimpleBLE warnings
if(TARGET simpleble)
  target_compile_options(simpleble PRIVATE -w)
endif()
set(simpleble_FOUND 1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Node: Camera publisher
add_executable(
  camera_publisher_node
  src/camera_publisher_node.cpp
)

# Node: Lane detector
add_executable(
  lane_detector_node
  src/lane_detector_node.cpp
)

# Node: Publish Bluetooth to ESP32
add_executable(
  publish_bt_esp32_node
  src/publish_bt_esp32_node.cpp
)

# Node: Publish Wi-Fi UDP to ESP32
add_executable(
  publish_wifi_esp32_node
  src/publish_wifi_esp32_node.cpp
)

# Node: Sensors receiver from ESP32 + BT
add_executable(
  sensors_receiver_bt_node
  src/sensors_receiver_bt_node.cpp
)

# Node: Sensors receiver from ESP32 + Wi-Fi
add_executable(
  sensors_receiver_wifi_node
  src/sensors_receiver_wifi_node.cpp
)

# Node: Decision maker
add_executable(
  decision_maker_node
  src/decision_maker_node.cpp
)

# Link libraries (+ OpenCV)
target_link_libraries(
  camera_publisher_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBS}
)
target_link_libraries(
  lane_detector_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json
)
target_link_libraries(
  publish_bt_esp32_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  simpleble::simpleble
)
target_link_libraries(
  publish_wifi_esp32_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)
target_link_libraries(
  sensors_receiver_bt_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  simpleble::simpleble
  nlohmann_json::nlohmann_json
)
target_link_libraries(
  sensors_receiver_wifi_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)
target_link_libraries(
  decision_maker_node  PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)

# Treat SimpleBLE includes as SYSTEM to suppress warnings
target_include_directories(publish_bt_esp32_node SYSTEM PRIVATE
  ${simpleble_SOURCE_DIR}/simpleble/include
)
target_include_directories(sensors_receiver_bt_node SYSTEM PRIVATE
  ${simpleble_SOURCE_DIR}/simpleble/include
)

# Install executable
install(TARGETS
  camera_publisher_node
  lane_detector_node
  publish_bt_esp32_node
  publish_wifi_esp32_node
  sensors_receiver_bt_node
  sensors_receiver_wifi_node
  decision_maker_node
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
