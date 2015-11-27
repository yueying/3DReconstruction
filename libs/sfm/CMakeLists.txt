INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/include")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/internal/ceres/miniglog")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/ceres/config")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/lemon")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/osi_clp/Clp/src/OsiClp")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/osi_clp/CoinUtils/src")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/osi_clp/Clp/src")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/osi_clp/Osi/src/Osi")
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	# Lib name
	sfm 		
	# Dependencies
	mvg_base
	mvg_image
	mvg_feature
	mvg_camera
	mvg_tracking
	mvg_multiview
	)

IF(BUILD_mvg_sfm) 

ENDIF(BUILD_mvg_sfm) 

