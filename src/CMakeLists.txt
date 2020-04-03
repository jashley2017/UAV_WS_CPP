cmake_minimum_required(VERSION 3.0)

project(uav)

add_compile_options(-std=c++11 -g)

add_subdirectory(vnproglib-1.1.4.0/cpp)
add_subdirectory(yaml-cpp)
add_subdirectory(uldaq)
add_subdirectory(logtk/logtk)

file(GLOB SRC_FILES *.cpp)

add_executable(uav_launch ${SRC_FILES})

include_directories(vnproglib-1.1.4.0/cpp/include yaml-cpp/include uldaq/src logtk/logtk)

target_link_libraries(uav_launch 
  yaml-cpp
  libvncxx
  uldaq
  logtk
)

install(TARGETS uav_launch DESTINATION ../../bin)
