cmake_minimum_required(VERSION 3.8)
project(sdv_lidar_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) 
find_package(tf2_ros REQUIRED) 

# External dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_geometry REQUIRED) 
find_package(geometry_msgs REQUIRED)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

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

add_executable(Lidar_Processing_node src/Processing_lidar.cpp)

ament_target_dependencies(Lidar_Processing_node rclcpp std_msgs pcl_conversions sensor_msgs message_filters vision_msgs image_geometry visualization_msgs tf2_ros geometry_msgs)
target_link_libraries(Lidar_Processing_node Eigen3::Eigen)
target_link_libraries(Lidar_Processing_node ${OpenCV_LIBS})

target_link_libraries(Lidar_Processing_node ${PCL_LIBRARIES})

install(TARGETS
  Lidar_Processing_node
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
