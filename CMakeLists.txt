cmake_minimum_required(VERSION 3.8)
project(codes)  

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
)

add_library(codes_component SHARED
  src/calculadoranode.cpp
)

ament_target_dependencies(codes_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
)

rclcpp_components_register_nodes(codes_component "calculadora::MainNode")

add_executable(codes_main
  src/calculadoramain.cpp
)

ament_target_dependencies(codes_main
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
)

target_link_libraries(codes_main
  codes_component
)

install(TARGETS
  codes_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  codes_main
  DESTINATION libexec/${PROJECT_NAME} 
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
