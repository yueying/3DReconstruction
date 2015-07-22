macro(add_base_srcs _PATH)
	LIST(APPEND base_EXTRA_SRCS			"${FBLIB_LIBS_ROOT}/base/src/${_PATH}/*.cpp" "${FBLIB_LIBS_ROOT}/base/include/fblib/${_PATH}/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"${_PATH}" "${_PATH}")
endmacro(add_base_srcs)

add_base_srcs(math)
add_base_srcs(random)
add_base_srcs(synch)
add_base_srcs(threads)
add_base_srcs(system)
add_base_srcs(utils)
#---------------------------------------------
# Macro declared in "DeclareFBLIBLib.cmake":
#---------------------------------------------
define_fblib_lib(
	base 		# Lib name
				# Dependencies: None
	)

IF(BUILD_fblib_base) 
	# Ignore precompiled headers in some sources:
	IF(FBLIB_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				PROPERTIES COMPILE_FLAGS "/Y-"
				)

		ENDIF (MSVC)
	ENDIF(FBLIB_ENABLE_PRECOMPILED_HDRS)

ENDIF(BUILD_fblib_base) 

