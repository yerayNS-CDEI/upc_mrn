cmake_minimum_required(VERSION 3.8)
project(upc_mrn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Own messages
set(msg_files
    "msg/Frontier.msg"
    "msg/Frontiers.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
                           ${msg_files}
                           DEPENDENCIES 
                           builtin_interfaces
                           geometry_msgs
                           std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# PubTwist
add_executable(pub_twist src/pub_twist.cpp)
ament_target_dependencies(pub_twist
                          rclcpp
                          geometry_msgs)

# OdomNew
add_executable(odom_new src/odom_new.cpp)
ament_target_dependencies(odom_new
                          rclcpp
                          tf2
                          tf2_geometry_msgs
                          nav_msgs
                          sensor_msgs)

# LaserProcessor
add_executable(laser_processor src/laser_processor.cpp)
ament_target_dependencies(laser_processor
                          rclcpp
                          tf2
                          tf2_geometry_msgs
                          visualization_msgs
                          nav_msgs
                          sensor_msgs)

# SafetyNode
add_executable(safety_node src/safety_node.cpp)
ament_target_dependencies(safety_node
                          rclcpp
                          geometry_msgs
                          sensor_msgs)

# FindFrontiers
add_executable(find_frontiers src/find_frontiers.cpp)
ament_target_dependencies(find_frontiers
                          rclcpp
                          geometry_msgs
                          nav_msgs
                          visualization_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(find_frontiers "${cpp_typesupport_target}")

# ExplorationRandom
add_executable(exploration_random src/exploration_random.cpp)
ament_target_dependencies(exploration_random
                          rclcpp
                          geometry_msgs
                          nav2_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          visualization_msgs
                          rclcpp_action
                          )
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(exploration_random "${cpp_typesupport_target}")

# ExplorationFirstFrontier
add_executable(exploration_first_frontier src/exploration_first_frontier.cpp)
ament_target_dependencies(exploration_first_frontier
                          rclcpp
                          geometry_msgs
                          nav2_msgs
                          tf2
                          tf2_ros
                          tf2_geometry_msgs
                          visualization_msgs
                          rclcpp_action
                          )
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(exploration_first_frontier "${cpp_typesupport_target}")

# Install nodes
install(TARGETS
        pub_twist
        odom_new
        laser_processor
        safety_node
        find_frontiers
        exploration_random
        exploration_first_frontier
        DESTINATION lib/${PROJECT_NAME})

# Install folders: launch, urdf, world, rviz
install(DIRECTORY
        config
        launch
        maps
        rviz
        urdf
        worlds
        DESTINATION share/${PROJECT_NAME}/
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
