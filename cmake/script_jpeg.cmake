# Check for system jpeglib:
# ===================================================
SET(CMAKE_FBLIB_HAS_JPEG 1)	# Always present: system or built-in
IF(MSVC)
	SET(CMAKE_FBLIB_HAS_JPEG_SYSTEM 0)
ELSE(MSVC)
	FIND_PACKAGE(JPEG)
	IF(JPEG_FOUND)
			#MESSAGE(STATUS "Found library: jpeg  - Include: ${JPEG_INCLUDE_DIR}")
			INCLUDE_DIRECTORIES("${JPEG_INCLUDE_DIR}")

			SET(JPEG_LIBS jpeg)  #APPEND_FBLIB_LIBS(jpeg)

			SET(CMAKE_FBLIB_HAS_JPEG_SYSTEM 1)
	ELSE(JPEG_FOUND)
			SET(CMAKE_FBLIB_HAS_JPEG_SYSTEM 0)
	ENDIF(JPEG_FOUND)
ENDIF(MSVC)
