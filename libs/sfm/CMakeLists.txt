INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/lemon")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/osi_clp/Clp/src/OsiClp")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/osi_clp/CoinUtils/src")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/osi_clp/Clp/src")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/osi_clp/Osi/src/Osi")
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	# Lib name
	sfm 		
	# Dependencies
	fblib_base
	fblib_image
	fblib_feature
	fblib_camera
	fblib_tracking
	fblib_multiview
	)

IF(BUILD_fblib_sfm) 

ENDIF(BUILD_fblib_sfm) 

