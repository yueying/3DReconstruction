PROJECT(png)

SET(PNG_SRCS
  pngget.c    pngrio.c    pngwrite.c  png.c       pngmem.c   pngrtran.c  pngtrans.c
  pngwtran.c  pngerror.c  pngpread.c  pngrutil.c  pngwutil.c 
  pngread.c   pngset.c    pngwio.c    pngconf.h   png.h
)

IF (WIN32)
  ADD_DEFINITIONS(-DPNG_STATIC)
  # TODO(keir): I don't know why, but CMake *refuses* to add this to the
  # command line flags. After much frustration, I am giving up and leaving the
  # compile errors. Please, someone who knows CMake, try to fix this.
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4996")
ENDIF (WIN32)

INCLUDE_DIRECTORIES(../zlib)
ADD_LIBRARY(png ${PNG_SRCS} )
TARGET_LINK_LIBRARIES(png zlib)

IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(png PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(png  PROPERTIES PROJECT_LABEL "(TEST) png")
endif(ENABLE_SOLUTION_FOLDERS)