cmake_minimum_required(VERSION 2.6.0)
cmake_policy(SET CMP0045 OLD)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(${CMAKE_INSTALL_PREFIX})
include_directories(${CMAKE_INSTALL_PREFIX}/include)

set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
include_directories(${EIGEN3_INCLUDE_DIR}) 
message("Eigen3 include dir" ${EIGEN3_INCLUDE_DIR})

find_package(PCL 1.5 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(OpenCV)

#add_executable(testMcCormickEnvelope testMcCormickEnvelope.cpp)
#target_link_libraries(testMcCormickEnvelope lcm drakeRBM drakeOptimization drakeJoints drakeCommon yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
#install(TARGETS testMcCormickEnvelope DESTINATION bin)


add_executable(kinectBridge kinectBridge.cpp)
target_link_libraries(kinectBridge ${PCL_LIBRARIES} lcm)