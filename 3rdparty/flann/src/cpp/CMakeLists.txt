
INCLUDE_DIRECTORIES(algorithms util nn .)

SET(SOURCES flann/flann.cpp)

ADD_LIBRARY(flann STATIC ${SOURCES})

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(flann PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(flann  PROPERTIES PROJECT_LABEL "(TEST) flann")
endif(ENABLE_SOLUTION_FOLDERS)