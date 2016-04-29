cmake_minimum_required(VERSION 2.8.3)
project(rva_find)

set(PROJECT_SRCS rva_find.cpp utilities.cpp getsourcecloud.cpp findplane.cpp findobject.cpp)

#Comment this if don't want debug
set(CMAKE_BUILD_TYPE Debug)

set(OpenCV_DIR "/usr/share/OpenCV")
find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(rva_find ${PROJECT_SRCS})

target_link_libraries(rva_find ${OpenCV_LIBS} ${PCL_LIBRARIES})
