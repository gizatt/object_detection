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

add_executable(testMcCormickEnvelope testMcCormickEnvelope.cpp)
target_link_libraries(testMcCormickEnvelope lcm drakeRBM drakeOptimization drakeJoints drakeCommon yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
install(TARGETS testMcCormickEnvelope DESTINATION bin)

add_executable(test3DRotationApprox test3DRotationApprox.cpp)
target_link_libraries(test3DRotationApprox lcm drakeRBM drakeOptimization drakeJoints drakeCommon drakeShapes yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
install(TARGETS test3DRotationApprox DESTINATION bin)

add_executable(miqp_point_model_detector miqp_point_model_detector.cpp)
target_link_libraries(miqp_point_model_detector lcm drakeRBM drakeMultibodyParsers drakeOptimization drakeJoints drakeCommon yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
install(TARGETS miqp_point_model_detector DESTINATION bin)

add_executable(milp_mesh_model_detector milp_mesh_model_detector.cpp)
target_link_libraries(milp_mesh_model_detector lcm drakeRBM drakeMultibodyParsers drakeShapes drakeOptimization drakeJoints drakeCommon yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
install(TARGETS milp_mesh_model_detector DESTINATION bin)

add_executable(milp_multiple_mesh_models_detector milp_multiple_mesh_models_detector.cpp)
target_link_libraries(milp_multiple_mesh_models_detector lcm drakeMultibodyParsers drakeRBM drakeOptimization drakeShapes drakeJoints drakeCommon yaml-cpp ${PCL_LIBRARIES} z ${OpenCV_LIBS} pthread)
install(TARGETS milp_multiple_mesh_models_detector DESTINATION bin)