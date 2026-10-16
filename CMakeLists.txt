cmake_minimum_required(VERSION 3.10)
project(ddmr_bridge CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Esta carpeta (bridge/) va dentro del repo TurtleTorqueNew, junto a include/ y src/.
get_filename_component(REPO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)

add_executable(bridge bridge.cpp ${REPO_ROOT}/src/motor_status.cpp)
target_include_directories(bridge PRIVATE ${REPO_ROOT})
target_compile_options(bridge PRIVATE -Wall)
