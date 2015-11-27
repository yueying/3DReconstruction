INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MVG application:  export_keypoints
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(export_keypoints)

#MESSAGE(STATUS "Makefile for application: /apps/export_keypoints ")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/apps/common/")
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(export_keypoints
               export_keypoints.cpp 
			    ${MVG_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "export_keypoints")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MVG_LINKER_LIBS} sift flann)

# Dependencies on MVG libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mvg_base mvg_image mvg_feature)

DeclareAppForInstall(${TMP_TARGET_NAME})

