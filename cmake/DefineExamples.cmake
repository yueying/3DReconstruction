# This file is included from the main CMakeLists.txt
#
SET( BUILD_EXAMPLES OFF CACHE BOOL "Build examples?")
IF(BUILD_EXAMPLES)
	SET(CMAKE_FBLIB_EXAMPLES_BASE_DIRECTORY "${CMAKE_SOURCE_DIR}/samples/")
	# Fix "\" --> "\\" for windows:
	string(REPLACE "\\" "\\\\" CMAKE_FBLIB_EXAMPLES_BASE_DIRECTORY ${CMAKE_FBLIB_EXAMPLES_BASE_DIRECTORY})

	#MESSAGE(STATUS "Parsing 'examples_config.h.in'")
	CONFIGURE_FILE("${CMAKE_SOURCE_DIR}/parse-files/examples_config.h.in" "${FBLIB_CONFIG_FILE_INCLUDE_DIR}/fblib/examples_config.h")

	# Generate CMakeLists.txt from the template project file for examples:
	MESSAGE(STATUS "Generating CMakefiles.txt for examples...")

	# ---------------------------------------------------------------
	#  MACRO for samples directories
	# ---------------------------------------------------------------
	MACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)
		# Convert CMAKE_EXAMPLE_DEPS -> CMAKE_EXAMPLE_DEPS_STRIP
		#          "fblib_xxx fblib_yyy" -> "xxx yyy"
		SET(CMAKE_EXAMPLE_DEPS_STRIP "")
		FOREACH(DEP ${CMAKE_EXAMPLE_DEPS})
			# Only for "fblib_XXX" libs:
			STRING(REGEX REPLACE "fblib_(.*)" "\\1" STRIP_DEP ${DEP})
			IF(NOT "${STRIP_DEP}" STREQUAL "")
				LIST(APPEND CMAKE_EXAMPLE_DEPS_STRIP ${STRIP_DEP})
			ENDIF(NOT "${STRIP_DEP}" STREQUAL "")
		ENDFOREACH(DEP)

		FOREACH(CMAKE_FBLIB_EXAMPLE_NAME ${LIST_EXAMPLES_IN_THIS_DIR})
			#MESSAGE(STATUS "Example: ${CMAKE_FBLIB_EXAMPLE_NAME}")
			# Generate project file:
			CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_template.txt.in ${CMAKE_SOURCE_DIR}/samples/${CMAKE_FBLIB_EXAMPLE_NAME}/CMakeLists.txt @ONLY)
			SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT}\nadd_subdirectory(${CMAKE_FBLIB_EXAMPLE_NAME})")
		ENDFOREACH(CMAKE_FBLIB_EXAMPLE_NAME)
	ENDMACRO(GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY)

	MACRO(ADD_SAMPLES_DIRECTORY dir)
		SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT}\nadd_subdirectory(${dir})")
	ENDMACRO(ADD_SAMPLES_DIRECTORY)
	# ---------------------------------------------------------------
	#  END OF MACRO for samples directories
	# ---------------------------------------------------------------

	# -----------------------------------------------------------------
	# This loop is generic, do not modify it...
	#  modify the above variable and/or the list_examples.txt files!
	# -----------------------------------------------------------------
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT "")

	# 添加example的依赖关系
	SET(LIST_EXAMPLES_IN_THIS_DIR
		learn_eigen
		svg_sample
		)
	SET(CMAKE_EXAMPLE_DEPS fblib_base)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${FBLIB_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()
	
	SET(LIST_EXAMPLES_IN_THIS_DIR
		exif_parsing
		image_test
		)
	SET(CMAKE_EXAMPLE_DEPS fblib_base fblib_image)
	SET(CMAKE_EXAMPLE_LINK_LIBS ${FBLIB_LINKER_LIBS})
	GENERATE_CMAKE_FILES_SAMPLES_DIRECTORY()

	# Generate the CMakeLists.txt in the "/samples" directory
	SET(CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS ${CMAKE_COMMANDS_INCLUDE_EXAMPLE_DIRS_ROOT})
	CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/samples/CMakeLists_list_template.txt.in "${CMAKE_SOURCE_DIR}/samples/CMakeLists.txt" )
	add_subdirectory(samples)
ENDIF(BUILD_EXAMPLES)
