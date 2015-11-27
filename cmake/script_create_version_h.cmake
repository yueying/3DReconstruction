# ----------------------------------------------------------------------------
#  Update the library version header file
#    FILE_TO_PARSE="SRC/include/mvg/MVG_version.h.in"
#    TARGET_FILE  ="MVG_version.h"
# ----------------------------------------------------------------------------
SET(CMAKE_MVG_COMPLETE_NAME "MVG ${CMAKE_MVG_VERSION_NUMBER_MAJOR}.${CMAKE_MVG_VERSION_NUMBER_MINOR}.${CMAKE_MVG_VERSION_NUMBER_PATCH}")
# Build a three digits version code, eg. 0.5.1 -> 051,  1.2.0 -> 120
SET(CMAKE_MVG_VERSION_CODE "0x${CMAKE_MVG_VERSION_NUMBER_MAJOR}${CMAKE_MVG_VERSION_NUMBER_MINOR}${CMAKE_MVG_VERSION_NUMBER_PATCH}")

CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/version.h.in" "${MVG_CONFIG_FILE_INCLUDE_DIR}/mvg/version.h")


# Prepare version.rc for Windows apps: 
IF (WIN32)
	configure_file(
		${MVG_SOURCE_DIR}/parse-files/version.rc.in
		${MVG_BINARY_DIR}/version.rc
		@ONLY)
	SET(MVG_VERSION_RC_FILE "${MVG_BINARY_DIR}/version.rc")
ELSE(WIN32)
	SET(MVG_VERSION_RC_FILE "")
ENDIF (WIN32)
