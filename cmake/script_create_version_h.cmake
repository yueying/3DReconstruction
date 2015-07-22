# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/fblib/FBLIB_version.h.in"
#    TARGET_FILE  ="FBLIB_version.h"
# ----------------------------------------------------------------------------
SET(CMAKE_FBLIB_COMPLETE_NAME "FBLIB ${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}.${CMAKE_FBLIB_VERSION_NUMBER_MINOR}.${CMAKE_FBLIB_VERSION_NUMBER_PATCH}")
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_FBLIB_VERSION_CODE "0x${CMAKE_FBLIB_VERSION_NUMBER_MAJOR}${CMAKE_FBLIB_VERSION_NUMBER_MINOR}${CMAKE_FBLIB_VERSION_NUMBER_PATCH}")

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${FBLIB_CONFIG_FILE_INCLUDE_DIR}/fblib/version.h")


# Prepare version.rc for Windows apps: 
IF (WIN32)
	configure_file(
		${FBLIB_SOURCE_DIR}/parse-files/version.rc.in
		${FBLIB_BINARY_DIR}/version.rc
		@ONLY)
	SET(FBLIB_VERSION_RC_FILE "${FBLIB_BINARY_DIR}/version.rc")
ELSE(WIN32)
	SET(FBLIB_VERSION_RC_FILE "")
ENDIF (WIN32)
