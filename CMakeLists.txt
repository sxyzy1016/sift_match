cmake_minimum_required(VERSION 3.7)
project(sift_match)

set(CMAKE_CXX_STANDARD 11)
find_package(Qt5Widgets REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)