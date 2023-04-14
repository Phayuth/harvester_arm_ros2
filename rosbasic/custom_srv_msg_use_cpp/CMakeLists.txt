cmake_minimum_required(VERSION 3.5)
project(custom_srv_msg_use_cpp)

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


# --------------------------INSTALL EXE----------------------------------------------------
find_package(custom_srv_msg REQUIRED)

add_executable(custom_msg_pub src/msg_pub_cpp.cpp)
ament_target_dependencies(custom_msg_pub rclcpp custom_srv_msg)

add_executable(custom_msg_sub src/msg_sub_cpp.cpp)
ament_target_dependencies(custom_msg_sub rclcpp custom_srv_msg)

add_executable(custom_server src/srv_server_cpp.cpp)
ament_target_dependencies(custom_server rclcpp custom_srv_msg)

add_executable(custom_client src/srv_client_cpp.cpp)
ament_target_dependencies(custom_client rclcpp custom_srv_msg)

install(TARGETS
  
  custom_msg_pub
  custom_msg_sub

  custom_client
  custom_server

  DESTINATION lib/${PROJECT_NAME})
# -----------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
