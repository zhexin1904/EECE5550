project(svd_icp)
# find_package(PCL)
find_package(PCL)
add_compile_options(-std=c++14)
add_definitions(-std=c++14)
include_directories(

    ${PCL_INCLUDE_DIRS}
    "/usr/include/pcl-1.10/"
    "/usr/include/eigens/")

# include(cmake/PCL.cmake)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(svd_icp svd_icp.cpp)

target_link_libraries(svd_icp ${PCL_LIBRARIES})

add_executable(kdTree_svd_icp kdTree_svd_icp.cpp)

target_link_libraries(kdTree_svd_icp ${PCL_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/../bin)