cmake_minimum_required(VERSION 3.0.0)

SET (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror -O3")

include_directories(
  ./
  ./lib/gl3w/include/
  /opt/local/include/
)

link_directories(
  /opt/local/lib/
)

add_executable(cube
  main.cpp
  gl-util/gl-common.cpp
  gl-util/glfw-util.cpp
  gl-util/shader.cpp
  gl-util/vertex-array.cpp
  gl-util/texture2d.cpp
  util/debug.cpp
  util/exceptions.cpp
  util/mat.cpp
  util/stopwatch.cpp
  lib/gl3w/src/gl3w.c
  sim/render.cpp
  sim/bodies.cpp
  sim/phys.cpp
)

FIND_LIBRARY(CORE_FOUNDATION_LIBRARY CoreFoundation)

target_link_libraries(cube
  glfw
  ${CORE_FOUNDATION_LIBRARY}
)
