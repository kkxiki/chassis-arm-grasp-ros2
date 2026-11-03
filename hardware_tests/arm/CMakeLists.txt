cmake_minimum_required(VERSION 3.10)
project(arm_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

find_package(Threads REQUIRED)

add_executable(arm_test
    arm_test.cpp
    ../../src/arm/tcp_arm_client.cpp
    ../../src/arm/arm_direct_controller.cpp
)

target_include_directories(arm_test PRIVATE
    ../../include
    ../../third_party
)

target_link_libraries(arm_test
    Threads::Threads
)
