cmake_minimum_required(VERSION 3.0)
project(astarPlanner)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 11)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
        ${Eigen3_INCLUDE_DIRS}
)

add_executable(mapProcess
        src/map.cpp
)
target_link_libraries(mapProcess  ${OpenCV_LIBS})

add_library(astar SHARED src/astar.cpp)
add_executable(astarPlanner 
        src/main.cpp
)
target_link_libraries(astarPlanner astar)

# visualize
add_executable(visualize src/visualize)
target_link_libraries(visualize ${OpenCV_LIBS})

# only used for test
add_executable(testMain src/testMain.cpp)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)