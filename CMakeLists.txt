cmake_minimum_required(VERSION 3.10)
project(agv_chassis_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

find_package(CURL REQUIRED)
find_package(Threads REQUIRED)

# ========== ROS2 依赖（视觉抓取）==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(service_interfaces REQUIRED)

# 编译可执行文件
add_executable(agv_demo
    src/main.cpp
    src/common/http_utils.cpp
    src/dispatch/dispatch_client.cpp
    src/chassis/chassis_controller.cpp
    src/arm/arm_controller.cpp
    src/task/task_manager.cpp
    src/arm/tcp_arm_client.cpp
    src/arm/arm_direct_controller.cpp
    src/vision/vision_grasp.cpp
)

# 头文件路径
target_include_directories(agv_demo PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

# 链接依赖库（注意：不能使用 PRIVATE 关键字，
# ament_target_dependencies 内部使用无关键字形式，混用会报错）
target_link_libraries(agv_demo
    CURL::libcurl
    Threads::Threads
)

# ROS2 接口依赖
ament_target_dependencies(agv_demo
    rclcpp
    vision_msgs
    service_interfaces
)

# ========== 安装到ROS2标准路径：lib/包名/ ==========
install(TARGETS agv_demo
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

ament_package()