LIST(APPEND multiview_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/ssba/Base/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/ssba/Geometry/*" "${FBLIB_SOURCE_DIR}/3rdparty/ssba/Math/*")
LIST(APPEND multiview_EXTRA_SRCS_NAME  "utils/ssba" "utils/ssba" "utils/ssba")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ssba/")

LIST(APPEND multiview_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/colamd/Include/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/colamd/Source/*.c" )
LIST(APPEND multiview_EXTRA_SRCS_NAME  "utils/colamd" "utils/colamd")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/colamd/Include")

INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/lemon")

add_definitions(-DV3DLIB_ENABLE_SUITESPARSE)
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	# Lib name
	multiview 		
	# Dependencies
	fblib_base
	fblib_image
	fblib_feature
	)


