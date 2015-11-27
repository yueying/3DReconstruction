macro(add_base_srcs _PATH)
	LIST(APPEND base_EXTRA_SRCS			"${MVG_LIBS_ROOT}/base/src/${_PATH}/*.cpp" "${MVG_LIBS_ROOT}/base/include/mvg/${_PATH}/*.h")
	LIST(APPEND base_EXTRA_SRCS_NAME 	"${_PATH}" "${_PATH}")
endmacro(add_base_srcs)

add_base_srcs(math)
add_base_srcs(random)
add_base_srcs(synch)
add_base_srcs(threads)
add_base_srcs(system)
add_base_srcs(utils)
#---------------------------------------------
# Macro declared in "DeclareMVGLib.cmake":
#---------------------------------------------
define_mvg_lib(
	base 		# Lib name
				# Dependencies: None
	)

IF(BUILD_mvg_base) 
	# Ignore precompiled headers in some sources:
	IF(MVG_ENABLE_PRECOMPILED_HDRS)
		IF (MSVC)
			set_source_files_properties(
				PROPERTIES COMPILE_FLAGS "/Y-"
				)

		ENDIF (MSVC)
	ENDIF(MVG_ENABLE_PRECOMPILED_HDRS)

ENDIF(BUILD_mvg_base) 

