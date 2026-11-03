cmake_minimum_required(VERSION 3.10)
project(chassis_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2")

find_package(CURL REQUIRED)
find_package(Threads REQUIRED)

add_executable(chassis_test
    chassis_test.cpp
    ../../src/common/http_utils.cpp
    ../../src/dispatch/dispatch_client.cpp
    ../../src/chassis/chassis_controller.cpp
)

target_include_directories(chassis_test PRIVATE
    ../../include
    ../../third_party
)

target_link_libraries(chassis_test
    CURL::libcurl
    Threads::Threads
)
