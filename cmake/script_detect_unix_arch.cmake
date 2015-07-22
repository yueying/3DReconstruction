if(UNIX)
	execute_process(COMMAND uname -m
		OUTPUT_VARIABLE CMAKE_FBLIB_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Architecture (uname -m): ${CMAKE_FBLIB_ARCH}")
	
	execute_process(COMMAND uname -s
		OUTPUT_VARIABLE CMAKE_FBLIB_KERNEL
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Kernel name (uname -s): ${CMAKE_FBLIB_KERNEL}")
endif(UNIX)
