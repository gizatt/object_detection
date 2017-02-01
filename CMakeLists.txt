cmake_minimum_required(VERSION 3.5)
project(object_detection)
include(ExternalProject)

ExternalProject_Add(pcl
	PREFIX ${CMAKE_BINARY_DIR}
	CMAKE_CACHE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pcl
	UPDATE_COMMAND ""
	INSTALL_COMMAND ""
)
if(NOT USE_SYSTEM_PCL)
  set(pcl_proj pcl)
  set(pcl_url http://github.com/pointcloudlibrary/pcl.git)
  #set(pcl_revision pcl-1.7.2) # this version introduces some missing openni pkg-config file bug
  set(pcl_revision pcl-1.7.1)
  set(pcl_depends flann Eigen_pod)
  set(pcl_external_args
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
    )
endif()

ExternalProject_Add(drake
	PREFIX ${CMAKE_BINARY_DIR}
	CMAKE_ARGS
		-DLCM_INCLUDE_DIRS:PATH=${CMAKE_BINARY_DIR}/include/lcmtypes
	CMAKE_CACHE_ARGS 
		-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR} 
		-DWITH_GUROBI:BOOL=TRUE
		-DWITH_SNOPT:BOOL=TRUE
		-DWITH_MOSEK:BOOL=TRUE
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drake
	INSTALL_DIR ${CMAKE_BINARY_DIR}
	UPDATE_COMMAND ""
	INSTALL_COMMAND "")

# This one is actually a pod, so set it up to build as one
ExternalProject_Add(drc_core_utils
	PREFIX ${CMAKE_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_PROJECT_drc_utils_INCLUDE:PATH=${CMAKE_SOURCE_DIR}/fix_include_dir.cmake
	CMAKE_CACHE_ARGS 
		-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR} 
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/drc_core_utils
	UPDATE_COMMAND ""
	INSTALL_DIR ${CMAKE_BINARY_DIR}
	INSTALL_COMMAND make install
	DEPENDS drake)