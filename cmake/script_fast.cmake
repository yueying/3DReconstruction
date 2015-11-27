# By default, compile this library if the directory exist:
# ------------------------------------------------------------------------
IF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/fast")
	SET( CMAKE_MVG_HAS_FAST 1)
ELSE(EXISTS "${MVG_LIBS_ROOT}/3rdparty/fast")
	SET( CMAKE_MVG_HAS_FAST 0)
ENDIF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/fast")

OPTION(DISABLE_FAST "Disable the fast library" "OFF")
MARK_AS_ADVANCED(DISABLE_FAST)
IF(DISABLE_FAST)
	SET(CMAKE_MVG_HAS_FAST 0)
ENDIF(DISABLE_FAST)
