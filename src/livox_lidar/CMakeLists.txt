cmake_minimum_required(VERSION 3.8)
project(livox_lidar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Check for livox_ros_driver2 (optional dependency)
set(HAVE_LIVOX_DRIVER FALSE)
find_package(livox_ros_driver2 QUIET)
if(livox_ros_driver2_FOUND)
  set(HAVE_LIVOX_DRIVER TRUE)
  message(STATUS "Found livox_ros_driver2, enabling full functionality")
else()
  message(WARNING "livox_ros_driver2 not found, some functionality will be disabled.\n"
                  "To enable full functionality, please install livox_ros_driver2 package.")
endif()

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolStamped.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/livox_lidar/livox_lidar_node.py
  src/livox_lidar/setup_livox.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Pass flag to Python code
if(HAVE_LIVOX_DRIVER)
  install(FILES
    config/config.yaml
    DESTINATION share/${PROJECT_NAME}/config
    RENAME config.yaml
  )
else()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/livox_driver_disabled.yaml
    "livox_driver_available: false\n")
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/livox_driver_disabled.yaml
    DESTINATION share/${PROJECT_NAME}/config
    RENAME config.yaml
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 