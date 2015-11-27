LIST(APPEND feature_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/daisy/src/*.cpp")
LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/daisy")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/daisy/include")

LIST(APPEND feature_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/fast/*.h" "${MVG_SOURCE_DIR}/3rdparty/fast/*.c")
LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/fast" "utils/fast")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/fast")

#LIST(APPEND feature_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/*.h" "${MVG_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/flann.cpp")
#LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/flann" "utils/flann")
INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/flann/src/cpp")

INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/sift/vl")

#LIST(APPEND feature_EXTRA_SRCS       "${MVG_SOURCE_DIR}/3rdparty/sift/vl/*.h" "${MVG_SOURCE_DIR}/3rdparty/sift/vl/*.c")
#LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/sift" "utils/sift")
#INCLUDE_DIRECTORIES("${MVG_SOURCE_DIR}/3rdparty/sift/vl")
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	# Lib name
	feature 		
	# Dependencies
	mvg_base
	mvg_image
	)

IF(BUILD_mvg_feature) 

ENDIF(BUILD_mvg_feature) 

