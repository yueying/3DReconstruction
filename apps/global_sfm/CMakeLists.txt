INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the MVG application:  global_sfm
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
			    ${MVG_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "global_sfm")

INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/lemon")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${MVG_LINKER_LIBS} lemon ceres lib_clp lib_CoinUtils lib_Osi lib_OsiClpSolver)



# Dependencies on MVG libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} mvg_base mvg_image mvg_sfm mvg_tracking)

DeclareAppForInstall(${TMP_TARGET_NAME})

