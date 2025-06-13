cmake_minimum_required(VERSION 3.8)
project(vision_tracking)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble")  # 添加 ROS Humble 根路径
list(APPEND CMAKE_MODULE_PATH "/opt/ros/humble/share/gazebo_ros/cmake")  # 添加 gazebo_ros 模块路径

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)

# ==== 头文件包含设置 ==== #
include_directories(include)  # 添加全局包含路径

# Create executables
add_executable(image_processor src/image_processor.cpp)
ament_target_dependencies(image_processor
  rclcpp cv_bridge image_transport sensor_msgs geometry_msgs)
target_link_libraries(image_processor ${OpenCV_LIBS})

# 为image_processor指定包含路径
target_include_directories(image_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(depth_processor src/depth_processor.cpp)
ament_target_dependencies(depth_processor
  rclcpp cv_bridge sensor_msgs geometry_msgs image_geometry tf2_ros tf2_geometry_msgs)
target_link_libraries(depth_processor ${OpenCV_LIBS})

# 为depth_processor指定包含路径
target_include_directories(depth_processor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller
  rclcpp geometry_msgs tf2_ros tf2_geometry_msgs)

# 为controller指定包含路径
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ==== 安装设置 ==== #
# 安装可执行文件
install(TARGETS 
  image_processor depth_processor controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装所有参数文件和目录
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# 安装其他资源
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()