cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.8)

# Set the project name
project(attach_shelf)

# Check if the compiler is GNU C++ or Clang, and add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)

# Generate the ROS interfaces for the project
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES builtin_interfaces
)

# Create the executable for the obstacle avoidance node
add_executable(obstacle_avoidance src/pre_approach.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp sensor_msgs geometry_msgs)

add_executable(pre_approach_v2 src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2 rclcpp sensor_msgs geometry_msgs std_srvs)

# Create the executable for the approach service server node
add_executable(approach_service_server src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server
  rclcpp
 sensor_msgs
  geometry_msgs
  std_srvs
  tf2_ros
  tf2
)

# Link against the typesupport library
rosidl_target_interfaces(approach_service_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executables and other files
install(TARGETS
  obstacle_avoidance
  approach_service_server
  pre_approach_v2
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch srv
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Create the package
ament_package()