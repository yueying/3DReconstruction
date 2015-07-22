# ----------------------------------------------------------------------------
# CMake file. Run with "cmake ." at the root directory.
# ----------------------------------------------------------------------------
project(ALL_FBLIB_APPS)

INCLUDE(../cmake/AssureCMakeRootFile.cmake) # Avoid user mistake in CMake source directory

# Enumerate all subdirs and process them:
list_subdirectories(LIST_FBLIB_APP_DIRS "${FBLIB_SOURCE_DIR}/apps")

foreach(dir ${LIST_FBLIB_APP_DIRS})
	SET(VAR_NAME "BUILD_APP_${dir}")
	SET(${VAR_NAME} ON CACHE BOOL "Build the app ${dir}")
	MARK_AS_ADVANCED(${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	endif (${VAR_NAME})
endforeach(dir)
