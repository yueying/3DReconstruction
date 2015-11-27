INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/lemon")
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	# Lib name
	tracking 		
	# Dependencies
	mvg_base
	mvg_image
	mvg_feature
	mvg_multiview
	mvg_camera
	mvg_reconstruction
	)

IF(BUILD_mvg_tracking) 

ENDIF(BUILD_mvg_tracking) 

