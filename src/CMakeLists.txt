cmake_minimum_required(VERSION 3.0.0)

SET (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -O3")

include_directories(./)

add_executable(cube
  main.cpp
)
