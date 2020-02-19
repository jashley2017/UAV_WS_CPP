cmake_minimum_required(VERSION 2.8.3)
project(vectornav)

add_compile_options(-std=c++11 -g)
project(vectornav)

add_subdirectory(vnproglib-1.1.4.0/cpp)
add_subdirectory(yaml-cpp)

include_directories(vnproglib-1.1.4.0/cpp/include yaml-cpp/include)

add_executable(vn_uav src/vectornav_async.cpp)

target_link_libraries(vn_uav
  libvncxx
  yaml-cpp
)
if(BUILD_WS)
target_link_libraries(vn_uav
  setupapi
)
endif(BUILD_WS)

install(TARGETS vn_uav DESTINATION ../../bin)
