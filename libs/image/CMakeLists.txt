LIST(APPEND image_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/jpeg/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/jpeg/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/jpeg" "utils/jpeg")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/jpeg")

LIST(APPEND image_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/png/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/png/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/png" "utils/png")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/png")

LIST(APPEND image_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/zlib/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/zlib/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/zlib" "utils/zlib")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/zlib")
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	# Lib name
	image 		
	# Dependencies
	fblib_base
	)

IF(BUILD_fblib_image) 

ENDIF(BUILD_fblib_image) 

