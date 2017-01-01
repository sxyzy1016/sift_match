cmake_minimum_required(VERSION 3.7)
project(sift_match)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

qt5_add_resources(QRC_FILES res.qrc)

set(SOURCE_FILES main.cpp mainwindow.cpp form.cpp SIFT.cpp match.cpp kdtree.cpp ${QRC_FILES})

add_executable(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)