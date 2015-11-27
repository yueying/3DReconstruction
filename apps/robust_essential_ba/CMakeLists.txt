INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MVG application:  robust_essential_ba
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(robust_essential_ba)

#MESSAGE(STATUS "Makefile for application: /apps/robust_essential_ba ")
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(robust_essential_ba
               robust_essential_ba.cpp
			    ${MVG_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "robust_essential_ba")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/sift/vl")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/config")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MVG_LINKER_LIBS} sift ceres)

# Dependencies on MVG libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mvg_base mvg_image mvg_feature mvg_multiview mvg_camera mvg_sfm)

DeclareAppForInstall(${TMP_TARGET_NAME})

