LIST(APPEND image_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/jpeg/*.h" "${MVG_SOURCE_DIR}/3rdparty/jpeg/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/jpeg" "utils/jpeg")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/jpeg")

LIST(APPEND image_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/png/*.h" "${MVG_SOURCE_DIR}/3rdparty/png/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/png" "utils/png")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/png")

LIST(APPEND image_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/zlib/*.h" "${MVG_SOURCE_DIR}/3rdparty/zlib/*.c")
LIST(APPEND image_EXTRA_SRCS_NAME  "utils/zlib" "utils/zlib")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/zlib")
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	# Lib name
	image 		
	# Dependencies
	mvg_base
	)

IF(BUILD_mvg_image) 

ENDIF(BUILD_mvg_image) 

