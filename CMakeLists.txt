cmake_minimum_required(VERSION 3.8)
project(vision_tracking)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Create executables
add_executable(image_processor src/image_processor.cpp)
ament_target_dependencies(image_processor
  rclcpp cv_bridge image_transport sensor_msgs geometry_msgs)
target_include_directories(image_processor PUBLIC include)
target_link_libraries(image_processor ${OpenCV_LIBS})

add_executable(depth_processor src/depth_processor.cpp)
ament_target_dependencies(depth_processor
  rclcpp cv_bridge sensor_msgs geometry_msgs image_geometry tf2_ros tf2_geometry_msgs)
target_include_directories(depth_processor PUBLIC include)
target_link_libraries(depth_processor ${OpenCV_LIBS})

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)
target_include_directories(controller PUBLIC include)

# Install executables
install(TARGETS image_processor depth_processor controller
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# Install RViz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()