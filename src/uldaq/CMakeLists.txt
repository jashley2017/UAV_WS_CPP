cmake_minimum_required(VERSION 3.0)
project(uldaq)
add_compile_options(-std=c++11 -g)

file(GLOB SRC_FILES src/*.cpp src/*h src/*/*.cpp src/*/*.h src/*/*/*.cpp src/*/*/*.h)

add_library(uldaq ${SRC_FILES})

target_link_libraries(uldaq
  usb-1.0
  )

