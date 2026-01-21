cmake_minimum_required(VERSION 3.16)
project(vvcm_fk)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DHAVE_CSTDDEF -D_USE_MATH_DEFINES)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  visualization_msgs
  interactive_markers
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vvcm_fk
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs visualization_msgs interactive_markers
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# =========================
# Eigen 3.4 via FetchContent
# =========================
include(FetchContent)

FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)

FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
  FetchContent_Populate(eigen)
endif()

# Header-only interface target
add_library(Eigen3::Eigen INTERFACE IMPORTED GLOBAL)
set_target_properties(Eigen3::Eigen PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
)

# ===============
# Library target
# ===============
add_library(vvcm_fk
  src/VVCM_FK.cpp
)

# IMPORTANT: PUBLIC so dependents (executables) inherit Eigen include dirs
target_link_libraries(vvcm_fk
  PUBLIC
    Eigen3::Eigen
  PRIVATE
    ${catkin_LIBRARIES}
)

# ==================
# Executable targets
# ==================
add_executable(VVCM_test src/VVCM_test.cpp)
target_link_libraries(VVCM_test
  PRIVATE
    vvcm_fk
)

add_executable(VVCM_test_ src/test/VVCM_FK_test.cpp)
target_link_libraries(VVCM_test_
  PRIVATE
    vvcm_fk
)
