cmake_minimum_required(VERSION 2.4)

project(V3D)

set (V3D_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set (V3D_INCLUDE_DIRS ${V3D_DIR}/.)

# TODO(keir): This may not work on Windows.
#add_definitions(-DV3DLIB_ENABLE_SUITESPARSE)

include_directories(${V3D_INCLUDE_DIRS} ${EXTRA_INC_DIRS})

IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

add_library(V3D
    Math/v3d_linear.h
    Math/v3d_mathutilities.h
    Math/v3d_optimization.h
    Math/v3d_optimization.cpp
    Geometry/v3d_cameramatrix.h
    Geometry/v3d_metricbundle.h
    Geometry/v3d_metricbundle.cpp)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(V3D PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(V3D  PROPERTIES PROJECT_LABEL "(TEST) V3D")
endif(ENABLE_SOLUTION_FOLDERS)