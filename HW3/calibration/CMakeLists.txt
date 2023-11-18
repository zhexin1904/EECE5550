project(calibration)
cmake_minimum_required(VERSION 3.0)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 11)
# set(OpenCV_DIR "/usr/local/share/OpenCV/OpenCVConfig.cmake")
# set(OpenCV_INCLUDE_DIRS "/usr/local/include/")
# set(OpenCV_LIBS "")
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake")

find_package(OpenCV REQUIRED
            PATHS "/usr/lib/x86_64-linux-gnu/cmake/opencv4"
            NO_DEFAULT_PATH
        )
include_directories(
    "/usr/include/eigen3"
    # "/usr/local/include/opencv2"
    ${OpenCV_INCLUDE_DIRS}

)
# find_package(OpenCV REQUIRED)

add_executable(calibration src/main.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
message(STATUS      "${OpenCV_DIR}")
message(STATUS      "${OpenCV_INCLUDE_DIRS}")
message(STATUS      "${OpenCV_LIBS}")