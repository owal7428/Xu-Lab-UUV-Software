cmake_minimum_required(VERSION 3.16)
project(XuLabHost)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SDL3 REQUIRED)

add_executable(Host Host.cpp)
target_link_libraries(Host PRIVATE SDL3::SDL3)
