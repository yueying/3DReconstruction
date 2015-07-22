# By default, compile this library if the directory exist:
# ------------------------------------------------------------------------
IF(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/flann")
	SET( CMAKE_FBLIB_HAS_FLANN 1)
ELSE(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/flann")
	SET( CMAKE_FBLIB_HAS_FLANN 0)
ENDIF(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/flann")

OPTION(DISABLE_FLANN "Disable the flann library" "OFF")
MARK_AS_ADVANCED(DISABLE_FLANN)
IF(DISABLE_FLANN)
	SET(CMAKE_FBLIB_HAS_FLANN 0)
ENDIF(DISABLE_FLANN)
