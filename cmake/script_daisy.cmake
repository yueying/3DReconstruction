# By default, compile this library if the directory exist:
# ------------------------------------------------------------------------
IF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/daisy")
	SET( CMAKE_MVG_HAS_DAISY 1)
ELSE(EXISTS "${MVG_LIBS_ROOT}/3rdparty/daisy")
	SET( CMAKE_MVG_HAS_DAISY 0)
ENDIF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/daisy")

OPTION(DISABLE_DAISY "Disable the daisy library" "OFF")
MARK_AS_ADVANCED(DISABLE_DAISY)
IF(DISABLE_DAISY)
	SET(CMAKE_MVG_HAS_DAISY 0)
ENDIF(DISABLE_DAISY)
