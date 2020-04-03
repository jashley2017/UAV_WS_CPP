cmake_minimum_required(VERSION 3.0)
project(logtk)
add_compile_options(-O3 -ansi -pedantic-errors -Wall -Wextra -Werror -Wno-long-long)

add_library(logtk logtk.cpp logtk.hpp)

include_directories(../strtk)

target_link_libraries(logtk
 /usr/lib stdc++ pthread boost_thread boost_filesystem boost_system
  )
