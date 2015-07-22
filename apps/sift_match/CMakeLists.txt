INCLUDE(../../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

#-----------------------------------------------------------------
# CMake file for the FBLIB application:  sift_match
#
#  Run with "cmake ." at the root directory
#
#  December 2015, fengbing <fengbing123@gmail.com>
#-----------------------------------------------------------------
PROJECT(sift_match)

#MESSAGE(STATUS "Makefile for application: /apps/sift_match ")
#INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/sift/vl")
# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(sift_match
               sift_match.cpp 
			    ${FBLIB_VERSION_RC_FILE})

SET(TMP_TARGET_NAME "sift_match")

LIST(APPEND feature_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/flann.cpp")
LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/flann" "utils/flann")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/sift/vl")
# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(${TMP_TARGET_NAME} ${FBLIB_LINKER_LIBS} sift flann)

# Dependencies on FBLIB libraries:
#  Just mention the top-level dependency, the rest will be detected automatically, 
#  and all the needed #include<> dirs added (see the script DeclareAppDependencies.cmake for further details)
DeclareAppDependencies(${TMP_TARGET_NAME} fblib_base fblib_image fblib_feature)

DeclareAppForInstall(${TMP_TARGET_NAME})

