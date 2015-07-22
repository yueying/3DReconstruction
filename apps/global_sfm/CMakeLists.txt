INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the FBLIB application:  global_sfm
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(global_sfm)

#MESSAGE(STATUS "Makefile for application: /apps/global_sfm ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(global_sfm
               global_sfm.cpp
			    ${FBLIB_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "global_sfm")

INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/lemon")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${FBLIB_LINKER_LIBS} lemon ceres lib_clp lib_CoinUtils lib_Osi lib_OsiClpSolver)



# Dependencies on FBLIB libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} fblib_base fblib_image fblib_sfm fblib_tracking)

DeclareAppForInstall(${TMP_TARGET_NAME})

