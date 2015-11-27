LIST(APPEND multiview_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/ssba/Base/*.h" "${MVG_SOURCE_DIR}/3rdparty/ssba/Geometry/*" "${MVG_SOURCE_DIR}/3rdparty/ssba/Math/*")
LIST(APPEND multiview_EXTRA_SRCS_NAME  "utils/ssba" "utils/ssba" "utils/ssba")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ssba/")

LIST(APPEND multiview_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/colamd/Include/*.h" "${MVG_SOURCE_DIR}/3rdparty/colamd/Source/*.c" )
LIST(APPEND multiview_EXTRA_SRCS_NAME  "utils/colamd" "utils/colamd")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/colamd/Include")

INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/lemon")

add_definitions(-DV3DLIB_ENABLE_SUITESPARSE)
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	# Lib name
	multiview 		
	# Dependencies
	mvg_base
	mvg_image
	mvg_feature
	mvg_camera
	)


