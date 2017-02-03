cmake_minimum_required(VERSION 3.5)
project(object_detection)
include(ExternalProject)

if(NOT USE_SYSTEM_PCL)
	ExternalProject_Add(pcl
		PREFIX ${CMAKE_BINARY_DIR}
		CMAKE_CACHE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR}
		SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pcl
		UPDATE_COMMAND ""
		INSTALL_COMMAND make install
	)
else()
  find_package(PCL 1.7 REQUIRED)
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

# This is added as an external project instead of a directory
# so I can force the pkgconfig files for the above projects to be completed
# before the CMake generation for the object_detection project is performed,
# so I can use those pkgconfig files in the CMake specifications for this project.
ExternalProject_Add(object_detection
	PREFIX ${CMAKE_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_PROJECT_object_detection_INCLUDE:PATH=${CMAKE_SOURCE_DIR}/fix_include_dir.cmake
	CMAKE_CACHE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_BINARY_DIR} -DCMAKE_BUILD_TYPE:STRING=Debug
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src
	UPDATE_COMMAND ""
	INSTALL_COMMAND make install 
	DEPENDS drake pcl)