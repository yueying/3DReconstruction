# ----------------------------------------------------------------------------
# Detect GNU version:
# ----------------------------------------------------------------------------
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} --version
		          OUTPUT_VARIABLE CMAKE_FBLIB_CLANG_VERSION_FULL
		          OUTPUT_STRIP_TRAILING_WHITESPACE)

	# Output in CMAKE_FBLIB_CLANG_VERSION_FULL: "X.Y"
	#  Look for the version number
	STRING(REGEX REPLACE ".* ([0-9]+.[0-9]+).*" "\\1" CMAKE_CLANG_REGEX_VERSION "${CMAKE_FBLIB_CLANG_VERSION_FULL}")

	# Split the three parts:
	STRING(REGEX MATCHALL "[0-9]+" CMAKE_FBLIB_CLANG_VERSIONS "${CMAKE_CLANG_REGEX_VERSION}")

	LIST(GET CMAKE_FBLIB_CLANG_VERSIONS 0 CMAKE_FBLIB_CLANG_VERSION_MAJOR)
	LIST(GET CMAKE_FBLIB_CLANG_VERSIONS 1 CMAKE_FBLIB_CLANG_VERSION_MINOR)

	SET(CMAKE_FBLIB_CLANG_VERSION ${CMAKE_FBLIB_CLANG_VERSION_MAJOR}${CMAKE_FBLIB_CLANG_VERSION_MINOR})

	IF($ENV{VERBOSE})
		MESSAGE(STATUS "clang --version: '${CMAKE_FBLIB_CLANG_VERSION_FULL}' -> Major=${CMAKE_FBLIB_CLANG_VERSION_MAJOR} Minor=${CMAKE_FBLIB_CLANG_VERSION_MINOR}")
	ENDIF($ENV{VERBOSE})
endif ()

