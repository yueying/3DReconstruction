# PNG
# ------------------------------------------

SET(CMAKE_MVG_HAS_PNG 0)
SET(CMAKE_MVG_HAS_PNG_SYSTEM 0)


set(png_USE_LAPACK_BLAS ON)
find_package(png QUIET NO_MODULE)  # 1st: Try to locate the *config.cmake file.
if(NOT png_FOUND)
	# Only use the Findpng.cmake module in Unix or if the user explicitly enables it in Windows:
	IF (WIN32)
		SET(DEFAULT_PNG_USE_FIND_MODULE "OFF")
	ELSE (WIN32)
		SET(DEFAULT_PNG_USE_FIND_MODULE "ON")
	ENDIF (WIN32)	
	SET(PNG_USE_FIND_MODULE ${DEFAULT_PNG_USE_FIND_MODULE} CACHE BOOL "Use CMake module to locate png?")
	
	IF(PNG_USE_FIND_MODULE)
        set(png_VERBOSE OFF)
        find_package(png QUIET) # 2nd: Use Findpng.cmake module
        include_directories(${png_INCLUDE_DIRS})
	ENDIF(PNG_USE_FIND_MODULE)
else(NOT png_FOUND)
		IF($ENV{VERBOSE})
			message(STATUS "Find png : include(${USE_png})")
		ENDIF($ENV{VERBOSE})
        include(${USE_png})
endif(NOT png_FOUND)

if(png_FOUND)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "png_LIBS: ${png_LIBRARIES}")
	ENDIF($ENV{VERBOSE})

	#APPEND_MVG_LIBS(${png_LIBRARIES})

	SET(CMAKE_MVG_HAS_PNG 1)
	SET(CMAKE_MVG_HAS_PNG_SYSTEM 1)
endif(png_FOUND)


