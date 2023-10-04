cmake_minimum_required(VERSION 3.0)
project(lieGroup)

find_package(Sophus)
find_package(Pangolin)

include_directories(/home/jason/EECE5550/lieGroup)
include_directories(/usr/include/eigen3)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Panolin_INCLUDE_DIRS})

add_library(visualize SHARED src/visualize.cpp )
# target_include_directories(visualize ${PROJECT_SOURCE_DIR}/include)

add_executable(basicSophus  ${PROJECT_SOURCE_DIR}/src/basicSophus.cpp)
target_link_libraries(basicSophus Sophus::Sophus)

add_executable(q3  ${PROJECT_SOURCE_DIR}/src/q3.cpp)
target_link_libraries(q3 Sophus::Sophus  visualize ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})

add_executable(q1  ${PROJECT_SOURCE_DIR}/src/q1.cpp)
target_link_libraries(q1 Sophus::Sophus)

add_executable(q4  ${PROJECT_SOURCE_DIR}/src/q4.cpp)
target_link_libraries(q4 Sophus::Sophus visualize ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})

# add_executable(visualize src/visualize.cpp)
# target_link_libraries(visualize ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})

# add_executable(visual src/visual.cpp)
# target_link_libraries(visual ${Pangolin_LIBRARIES} ${FMT_LIBRARIES})


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
