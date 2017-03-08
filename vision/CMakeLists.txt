cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0045 OLD)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB capturePointCloudFrame
    "*.h"
    "*.hpp"
    "*.cpp"
)

file(GLOB processDepthStream
    "*.h"
    "*.hpp"
    "*.cpp"
)

list(REMOVE_ITEM capturePointCloudFrame ${CMAKE_CURRENT_SOURCE_DIR}/processDepthStream.cpp)

list(REMOVE_ITEM processDepthStream ${CMAKE_CURRENT_SOURCE_DIR}/capturePointCloudFrame.cpp)


add_executable(capturePointCloudFrame ${capturePointCloudFrame})
target_link_libraries(capturePointCloudFrame ${PCL_LIBRARIES} lcm)


add_executable(processDepthStream ${processDepthStream})
target_link_libraries(processDepthStream ${PCL_LIBRARIES} lcm)