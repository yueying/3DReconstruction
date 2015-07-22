INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the FBLIB application:  export_tracks
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(export_tracks)

#MESSAGE(STATUS "Makefile for application: /apps/export_tracks ")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/apps/common/")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/lemon")
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(export_tracks
               export_tracks.cpp 
			    ${FBLIB_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "export_tracks")

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${FBLIB_LINKER_LIBS} lemon)

# Dependencies on FBLIB libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} fblib_base fblib_image fblib_feature fblib_tracking)

DeclareAppForInstall(${TMP_TARGET_NAME})

