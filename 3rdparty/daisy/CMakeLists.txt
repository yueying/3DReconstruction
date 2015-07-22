project(daisy)

include_directories(include)

# Make the headers appear in IDEs.
file(GLOB daisy_hdrs include/kutility/*.h include/daisy/*.h)

ADD_LIBRARY(daisy
	STATIC
    src/daisy.cpp
    src/corecv.cpp
    src/image_io_bmp.cpp
    src/image_io_pnm.cpp
    src/image_io_png.cpp
    src/image_io_jpeg.cpp
    src/image_manipulation.cpp
    src/progress_bar.cpp
    src/general.cpp
    src/interaction.cpp
)


IF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND UNIX)


if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(daisy PROPERTIES FOLDER "3rd party")
else(ENABLE_SOLUTION_FOLDERS)
	SET_TARGET_PROPERTIES(daisy  PROPERTIES PROJECT_LABEL "(TEST) daisy")
endif(ENABLE_SOLUTION_FOLDERS)
