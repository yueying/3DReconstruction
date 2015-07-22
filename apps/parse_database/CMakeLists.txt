INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the FBLIB application:  parse_database
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(parse_database)

#MESSAGE(STATUS "Makefile for application: /apps/parse_database ")

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(parse_database
               parse_database.cpp
			    ${FBLIB_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "parse_database")



# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${FBLIB_LINKER_LIBS})

# Dependencies on FBLIB libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} fblib_base fblib_image)

DeclareAppForInstall(${TMP_TARGET_NAME})

