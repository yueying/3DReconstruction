INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/lemon")
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	# Lib name
	tracking 		
	# Dependencies
	fblib_base
	fblib_image
	fblib_feature
	fblib_multiview
	fblib_camera
	fblib_reconstruction
	)

IF(BUILD_fblib_tracking) 

ENDIF(BUILD_fblib_tracking) 

