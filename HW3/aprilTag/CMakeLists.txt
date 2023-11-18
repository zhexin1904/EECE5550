project(aprilTag)
cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 14)


find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})
set(GTSAM_LIBRARIES gtsam)   

find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)

find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")


# find_package( apriltag REQUIRED)
# include_directories( ${apriltag_INCLUDE_DIRS} )

# find_package( AprilTags REQUIRED)
# include_directories( ${AprilTags_INCLUDE_DIRS} )
include_directories("/usr/local/include")
# include_directories( ${apriltag_INCLUDE_DIRS}/apriltag)
# set(apriltag_DIR "/usr/local/apriltag/share/apriltag/cmake/apriltagConfig.cmake")

# add_executable(test_apriltag src/apriltag_test.cpp)
# target_link_libraries(test_apriltag ${Boost_LIBRARIES} gtsam ${OpenCV_LIBS} "/usr/local/lib/libapriltags.so") 

add_executable(apriltag_pnp src/apriltag_pnp.cpp)
target_link_libraries(apriltag_pnp ${Boost_LIBRARIES} gtsam ${OpenCV_LIBS} "/usr/local/lib/libapriltags.so") 

# message(STATUS "apriltag_include dir:${AprilTags_INCLUDE_DIRS}")
# message(STATUS "apriltag_dir:${AprilTags_DIR}")
# message(STATUS "apriltag_libs:${AprilTags_LIBS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)