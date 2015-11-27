# By default, compile this library if the directory exist:
# ------------------------------------------------------------------------
IF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/ssba")
	SET( CMAKE_MVG_HAS_SSBA 1)
ELSE(EXISTS "${MVG_LIBS_ROOT}/3rdparty/ssba")
	SET( CMAKE_MVG_HAS_SSBA 0)
ENDIF(EXISTS "${MVG_LIBS_ROOT}/3rdparty/ssba")

OPTION(DISABLE_SSBA "Disable the ssba library" "OFF")
MARK_AS_ADVANCED(DISABLE_SSBA)
IF(DISABLE_SSBA)
	SET(CMAKE_MVG_HAS_SSBA 0)
ENDIF(DISABLE_SSBA)
