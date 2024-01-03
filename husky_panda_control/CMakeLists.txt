cmake_minimum_required(VERSION 3.10)

project(husky_panda)

set(CMAKE_CXX_STANDARD 11)
set(OpenGL_GL_PREFERENCE LEGACY)

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(mujoco REQUIRED)
find_package(RBDL REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  include
)

add_library(${PROJECT_NAME}_lib
  src/mujoco_bridge.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp src/mujoco_bridge.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  ${RBDL_LIBRARY}
  # pthread
  glfw
  mujoco
)
