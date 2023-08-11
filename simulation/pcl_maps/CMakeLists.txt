cmake_minimum_required(VERSION 3.8)
project(pcl_maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED) 

find_package(PCL REQUIRED COMPONENTS common io)

find_package(PCL 1.1 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_map_node src/pcl_map_node.cpp)
target_include_directories(pcl_map_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(pcl_map_node
  rclcpp
  std_msgs 
  sensor_msgs 
  image_transport 
  cv_bridge
  pcl_conversions
  geometry_msgs
  visualization_msgs
)

target_link_libraries(pcl_map_node ${PCL_LIBRARIES})


target_compile_features(pcl_map_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS pcl_map_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
