cmake_minimum_required(VERSION 3.0)

option(BUILD_WS "Build the Windows executable instead of Linux" OFF)

if(BUILD_WS)
  set(CMAKE_SYSTEM_NAME Windows)


  set(TOOLCHAIN_PREFIX x86_64-w64-mingw32)
  set(CMAKE_C_COMPILER /usr/bin/${TOOLCHAIN_PREFIX}-gcc)
  set(CMAKE_CXX_COMPILER /usr/bin/${TOOLCHAIN_PREFIX}-g++)
  set(CMAKE_RC_COMPILER /usr/bin/${TOOLCHAIN_PREFIX}-windres)
  set(CMAKE_RANLIB /usr/bin/${TOOLCHAIN_PREFIX}-ranlib)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -static -static-libgcc -static-libstdc++")
  set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

  # target environment on the build host system
  set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX} /usr/lib/gcc/${TOOLCHAIN_PREFIX}/7.3-posix)
  set(CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES /usr/lib/gcc/${TOOLCHAIN_PREFIX}/7.3-posix)


  # modify default behavior of FIND_XXX() commands to
  # search for headers/libs in the target environment and
  # search for programs in the build host environment
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
endif(BUILD_WS)

project(uav)
add_subdirectory(vectornav)
