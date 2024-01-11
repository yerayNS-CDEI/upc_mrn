cmake_minimum_required(VERSION 3.8)
project(upc_mrn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Node prova
add_executable(node_prova src/node_prova.cpp)
ament_target_dependencies(node_prova rclcpp std_msgs)

# PubTwist
add_executable(pub_twist src/pub_twist.cpp)
ament_target_dependencies(pub_twist rclcpp geometry_msgs)

# Install nodes
install(TARGETS
        node_prova 
        pub_twist
        DESTINATION lib/${PROJECT_NAME})

# Install folders: launch, urdf, world, rviz
install(DIRECTORY
        launch
        urdf
        rviz
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
