# ----------------------------------------------------------------------------
#   Generate the FBLIBConfig.cmake & configure files
# ----------------------------------------------------------------------------
# Create the code fragment: "DECLARE_LIBS_DEPS", for usage below while
#  generating "FBLIBConfig.cmake"
SET(DECLARE_LIBS_DEPS "")
FOREACH(_LIB ${ALL_FBLIB_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	SET(DECLARE_LIBS_DEPS "${DECLARE_LIBS_DEPS} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_DEPS\" \"${_LIB_DEP}\")\n")
ENDFOREACH(_LIB)

# Create the code fragment: "DECLARE_LIBS_HDR_ONLY", for usage below while
#  generating "FBLIBConfig.cmake"
SET(DECLARE_LIBS_HDR_ONLY "")
FOREACH(_LIB ${ALL_FBLIB_LIBS})
	get_property(_LIB_HDR_ONLY GLOBAL PROPERTY "${_LIB}_LIB_IS_HEADERS_ONLY")
	SET(DECLARE_LIBS_HDR_ONLY "${DECLARE_LIBS_HDR_ONLY} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_IS_HEADERS_ONLY\" \"${_LIB_HDR_ONLY}\")\n")
ENDFOREACH(_LIB)

# ----------------------------------------------------------------------------
#   Generate the FBLIBConfig.cmake file
# ----------------------------------------------------------------------------
SET(THE_FBLIB_SOURCE_DIR "${FBLIB_SOURCE_DIR}")
SET(THE_FBLIB_LIBS_INCL_DIR "${THE_FBLIB_SOURCE_DIR}/libs")
SET(THE_CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}")
SET(THE_FBLIB_CONFIG_FILE_INCLUDE_DIR "${FBLIB_CONFIG_FILE_INCLUDE_DIR}")
SET(FBLIB_CONFIGFILE_IS_INSTALL 0)

CONFIGURE_FILE(
	"${FBLIB_SOURCE_DIR}/parse-files/FBLIBConfig.cmake.in"
    "${FBLIB_BINARY_DIR}/FBLIBConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding FBLIB, e.g. find_package(FBLIB 1.0.0 EXACT)
CONFIGURE_FILE(
	"${FBLIB_SOURCE_DIR}/parse-files/FBLIBConfig-version.cmake.in" 
	"${CMAKE_BINARY_DIR}/FBLIBConfig-version.cmake" IMMEDIATE @ONLY)

# ----------------------------------------------------------------------------
#   Generate the FBLIBConfig.cmake file for unix
#      installation in CMAKE_INSTALL_PREFIX
# ----------------------------------------------------------------------------
SET(FBLIB_CONFIGFILE_IS_INSTALL 1)
IF(WIN32)
	SET(THE_FBLIB_SOURCE_DIR "\${THIS_FBLIB_CONFIG_PATH}")
	SET(THE_FBLIB_LIBS_INCL_DIR "${THE_FBLIB_SOURCE_DIR}/libs")
	SET(THE_CMAKE_BINARY_DIR "\${THIS_FBLIB_CONFIG_PATH}")
	SET(THE_FBLIB_CONFIG_FILE_INCLUDE_DIR "\${THIS_FBLIB_CONFIG_PATH}/include/fblib/fblib_config/")
ELSE(WIN32)
	# Unix install. This .cmake file will end up in /usr/share/fblib/FBLIBConfig.cmake :
	IF (CMAKE_FBLIB_USE_DEB_POSTFIXS)
		# We're building a .deb package: DESTDIR is NOT the final installation directory:
		SET(THE_FBLIB_SOURCE_DIR "/usr")
		SET(THE_FBLIB_LIBS_INCL_DIR "${THE_FBLIB_SOURCE_DIR}/include/fblib")
		SET(THE_CMAKE_BINARY_DIR "/usr")
		SET(THE_FBLIB_CONFIG_FILE_INCLUDE_DIR "/usr/include/fblib/fblib_config/")
	ELSE(CMAKE_FBLIB_USE_DEB_POSTFIXS)
		# Normal case: take the desired installation directory
		SET(THE_FBLIB_SOURCE_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_FBLIB_LIBS_INCL_DIR "${THE_FBLIB_SOURCE_DIR}/include/fblib")
		SET(THE_CMAKE_BINARY_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_FBLIB_CONFIG_FILE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/fblib/fblib_config/")
	ENDIF(CMAKE_FBLIB_USE_DEB_POSTFIXS)
ENDIF(WIN32)

CONFIGURE_FILE(
	"${FBLIB_SOURCE_DIR}/parse-files/FBLIBConfig.cmake.in"  
	"${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding FBLIB, e.g. find_package(FBLIB 1.0.0 EXACT)
CONFIGURE_FILE(
	"${FBLIB_SOURCE_DIR}/parse-files/FBLIBConfig-version.cmake.in" 
	"${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig-version.cmake" IMMEDIATE @ONLY)
