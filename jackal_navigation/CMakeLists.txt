cmake_minimum_required(VERSION 3.5)
project(jackal_navigation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

add_executable(world_odom_tf_pub src/world_odom_tf_pub.cpp)
ament_target_dependencies(world_odom_tf_pub rclcpp geometry_msgs nav2_msgs tf2 tf2_ros)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS world_odom_tf_pub
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
