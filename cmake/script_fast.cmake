# By default, compile this library if the directory exist:
# ------------------------------------------------------------------------
IF(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/fast")
	SET( CMAKE_FBLIB_HAS_FAST 1)
ELSE(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/fast")
	SET( CMAKE_FBLIB_HAS_FAST 0)
ENDIF(EXISTS "${FBLIB_LIBS_ROOT}/3rdparty/fast")

OPTION(DISABLE_FAST "Disable the fast library" "OFF")
MARK_AS_ADVANCED(DISABLE_FAST)
IF(DISABLE_FAST)
	SET(CMAKE_FBLIB_HAS_FAST 0)
ENDIF(DISABLE_FAST)
