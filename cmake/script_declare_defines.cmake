# ----------------------------------------------------------------------------
# Transform the variables FBLIB_XXX="ON/OFF" to CMAKE_FBLIB_XXX="1/0"
# ----------------------------------------------------------------------------
MACRO(CREATE_CMAKE_FBLIB_DEFINE defName)
	IF(${defName} MATCHES "ON")
		SET(CMAKE_${defName} "1")
	ELSE(${defName} MATCHES "ON")
		SET(CMAKE_${defName} "0")
	ENDIF(${defName} MATCHES "ON")
ENDMACRO(CREATE_CMAKE_FBLIB_DEFINE)

CREATE_CMAKE_FBLIB_DEFINE(FBLIB_ALWAYS_CHECKS_DEBUG)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_ALWAYS_CHECKS_DEBUG_MATRICES)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_HAS_BUMBLEBEE)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_HAS_SVS)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_HAS_ASSERT)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_HAS_STACKED_EXCEPTIONS)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_ENABLE_EMBEDDED_GLOBAL_PROFILER)
CREATE_CMAKE_FBLIB_DEFINE(FBLIB_HAS_ASIAN_FONTS)
CREATE_CMAKE_FBLIB_DEFINE(CMAKE_FBLIB_HAS_INOTIFY)
