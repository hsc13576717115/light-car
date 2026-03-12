cmake_minimum_required(VERSION 3.12)
project(gimbal_controller)

# ====================== 通用设置 ======================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 让 CMake 优先使用本地 RKNN 库，并写进 rpath
list(PREPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${RKNN_LIBRARY_DIR}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# ====================== ROS 2 包 ======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# ====================== OpenCV 4 ======================
find_package(OpenCV REQUIRED COMPONENTS
  core imgproc highgui calib3d videoio tracking)

# ====================== RKNN ======================
set(RKNN_INCLUDE_DIR "/home/orangepi/rknn-toolkit2-master/rknpu2/runtime/Linux/librknn_api/include")
set(RKNN_LIBRARY_DIR "/home/orangepi/rknn-toolkit2-master/rknpu2/runtime/Linux/librknn_api/aarch64")
find_library(RKNN_LIBRARY rknnrt PATHS ${RKNN_LIBRARY_DIR})

# ====================== 头文件 & 库目录 ======================
include_directories(
  include
  include/gimbal_controller
  ${OpenCV_INCLUDE_DIRS}
  ${RKNN_INCLUDE_DIR}
)
link_directories(${RKNN_LIBRARY_DIR})

# ====================== 源文件 ======================
set(GIMBAL_SRC
  src/test_damiao.cpp
)
set(YOLO_SRC
  src/gimbal_yolo_node.cpp
)

# ====================== 可执行文件 ======================
# 1) 旧节点
add_executable(gimbal_control_node ${GIMBAL_SRC})
ament_target_dependencies(gimbal_control_node
  rclcpp rclcpp_components std_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
find_library(GPIOD_LIB gpiod)
if(GPIOD_LIB)
  target_link_libraries(gimbal_control_node ${GPIOD_LIB})
endif()

# 2) YOLO 融合节点（含 MJPG 兼容）
add_executable(gimbal_yolo_node ${YOLO_SRC})
ament_target_dependencies(gimbal_yolo_node
  rclcpp std_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs)
target_link_libraries(gimbal_yolo_node
  ${OpenCV_LIBS}
  ${RKNN_LIBRARY}
  pthread atomic)

# ====================== 安装 ======================
install(TARGETS
  gimbal_control_node
  gimbal_yolo_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

# ====================== ARM64 额外原子库 ======================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_libraries(gimbal_yolo_node atomic)
endif()

# ====================== 包导出 ======================
ament_package()