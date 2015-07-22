LIST(APPEND feature_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/daisy/src/*.cpp")
LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/daisy")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/daisy/include")

LIST(APPEND feature_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/fast/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/fast/*.c")
LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/fast" "utils/fast")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/fast")

#LIST(APPEND feature_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp/flann/flann.cpp")
#LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/flann" "utils/flann")
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/flann/src/cpp")

INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/sift/vl")

#LIST(APPEND feature_EXTRA_SRCS       "${FBLIB_SOURCE_DIR}/3rdparty/sift/vl/*.h" "${FBLIB_SOURCE_DIR}/3rdparty/sift/vl/*.c")
#LIST(APPEND feature_EXTRA_SRCS_NAME  "utils/sift" "utils/sift")
#INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/sift/vl")
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	# Lib name
	feature 		
	# Dependencies
	fblib_base
	fblib_image
	)

IF(BUILD_fblib_feature) 

ENDIF(BUILD_fblib_feature) 

