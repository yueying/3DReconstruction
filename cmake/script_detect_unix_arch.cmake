if(UNIX)
	execute_process(COMMAND uname -m
		OUTPUT_VARIABLE CMAKE_MVG_ARCH
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Architecture (uname -m): ${CMAKE_MVG_ARCH}")
	
	execute_process(COMMAND uname -s
		OUTPUT_VARIABLE CMAKE_MVG_KERNEL
                OUTPUT_STRIP_TRAILING_WHITESPACE)
	message(STATUS "Kernel name (uname -s): ${CMAKE_MVG_KERNEL}")
endif(UNIX)
