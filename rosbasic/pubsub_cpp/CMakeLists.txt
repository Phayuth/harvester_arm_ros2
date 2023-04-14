cmake_minimum_required(VERSION 3.5)
project(pubsub_cpp)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED) # add this for new dependency
find_package(std_msgs REQUIRED) # add this for new dependendy


# build exe file for cpp
add_executable(pub_cpp src/pub_cpp.cpp)
ament_target_dependencies(pub_cpp rclcpp std_msgs) # link exe to library dependency

add_executable(sub_cpp src/sub_cpp.cpp)
ament_target_dependencies(sub_cpp rclcpp std_msgs) # link exe to library dependency


install(TARGETS 
        pub_cpp 
        sub_cpp
        DESTINATION lib/${PROJECT_NAME})


ament_package()