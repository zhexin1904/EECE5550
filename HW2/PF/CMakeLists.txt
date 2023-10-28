project(HW2_3)

cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 11)

find_package(Sophus)
include_directories(${Sophus_ICLUDE_DIRS})
include_directories("/usr/include/eigen")
include_directories("/home/jason/EECE5550/hw/HW2-3")

add_executable(SE2_motion ${PROJECT_SOURCE_DIR}/src/robot.cpp)
target_link_libraries(SE2_motion Sophus::Sophus)

add_executable(SE2_pf ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(SE2_pf Sophus::Sophus)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)