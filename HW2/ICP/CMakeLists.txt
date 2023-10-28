cmake_minimum_required(VERSION 3.0)

project(hw2)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g")
set(CMAKE_CXX_STANDARD 11)


add_subdirectory(src)
