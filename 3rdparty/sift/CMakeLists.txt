PROJECT(sift)

include_directories(include)
INCLUDE_DIRECTORIES("${FBLIB_SOURCE_DIR}/3rdparty/sift/vl")

file(GLOB include/sift/vl/*.h)

ADD_LIBRARY(sift 
	STATIC
	vl/generic.h
  vl/imopv_sse2.h
  vl/sift.h
  vl/imopv.h
  vl/mathop_sse2.h
  vl/host.h
  vl/mathop.h
  vl/random.h
	vl/generic.c
  vl/imopv_sse2.c
  vl/sift.c
  vl/imopv.c
  vl/mathop_sse2.c
  vl/host.c
  vl/mathop.c
  vl/random.c
	)

SET_SOURCE_FILES_PROPERTIES(sift PROPERTIES LANGUAGE C)


ADD_DEFINITIONS(-D__SSE2__)

ADD_DEFINITIONS(-DUSE_SSE)


if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(sift PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(sift  PROPERTIES PROJECT_LABEL "(TEST) sift")
endif(ENABLE_SOLUTION_FOLDERS)



