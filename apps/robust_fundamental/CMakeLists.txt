INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MVG application:  robust_fundamental
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(robust_fundamental)

#MESSAGE(STATUS "Makefile for application: /apps/robust_fundamental ")
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(robust_fundamental
               robust_fundamental.cpp
			    ${MVG_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "robust_fundamental")

INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/sift/vl")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MVG_LINKER_LIBS} sift)

# Dependencies on MVG libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mvg_base mvg_image mvg_feature mvg_multiview mvg_camera)

DeclareAppForInstall(${TMP_TARGET_NAME})

