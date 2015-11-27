# ----------------------------------------------------------------------------
#   Generate the MVGConfig.cmake & configure files
# ----------------------------------------------------------------------------
# Create the code fragment: "DECLARE_LIBS_DEPS", for usage below while
#  generating "MVGConfig.cmake"
SET(DECLARE_LIBS_DEPS "")
FOREACH(_LIB ${ALL_MVG_LIBS})
	get_property(_LIB_DEP GLOBAL PROPERTY "${_LIB}_LIB_DEPS")
	SET(DECLARE_LIBS_DEPS "${DECLARE_LIBS_DEPS} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_DEPS\" \"${_LIB_DEP}\")\n")
ENDFOREACH(_LIB)

# Create the code fragment: "DECLARE_LIBS_HDR_ONLY", for usage below while
#  generating "MVGConfig.cmake"
SET(DECLARE_LIBS_HDR_ONLY "")
FOREACH(_LIB ${ALL_MVG_LIBS})
	get_property(_LIB_HDR_ONLY GLOBAL PROPERTY "${_LIB}_LIB_IS_HEADERS_ONLY")
	SET(DECLARE_LIBS_HDR_ONLY "${DECLARE_LIBS_HDR_ONLY} set_property(GLOBAL PROPERTY \"${_LIB}_LIB_IS_HEADERS_ONLY\" \"${_LIB_HDR_ONLY}\")\n")
ENDFOREACH(_LIB)

# ----------------------------------------------------------------------------
#   Generate the MVGConfig.cmake file
# ----------------------------------------------------------------------------
SET(THE_MVG_SOURCE_DIR "${MVG_SOURCE_DIR}")
SET(THE_MVG_LIBS_INCL_DIR "${THE_MVG_SOURCE_DIR}/libs")
SET(THE_CMAKE_BINARY_DIR "${CMAKE_BINARY_DIR}")
SET(THE_MVG_CONFIG_FILE_INCLUDE_DIR "${MVG_CONFIG_FILE_INCLUDE_DIR}")
SET(MVG_CONFIGFILE_IS_INSTALL 0)

CONFIGURE_FILE(
	"${MVG_SOURCE_DIR}/parse-files/MVGConfig.cmake.in"
    "${MVG_BINARY_DIR}/MVGConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding MVG, e.g. find_package(MVG 1.0.0 EXACT)
CONFIGURE_FILE(
	"${MVG_SOURCE_DIR}/parse-files/MVGConfig-version.cmake.in" 
	"${CMAKE_BINARY_DIR}/MVGConfig-version.cmake" IMMEDIATE @ONLY)

# ----------------------------------------------------------------------------
#   Generate the MVGConfig.cmake file for unix
#      installation in CMAKE_INSTALL_PREFIX
# ----------------------------------------------------------------------------
SET(MVG_CONFIGFILE_IS_INSTALL 1)
IF(WIN32)
	SET(THE_MVG_SOURCE_DIR "\${THIS_MVG_CONFIG_PATH}")
	SET(THE_MVG_LIBS_INCL_DIR "${THE_MVG_SOURCE_DIR}/libs")
	SET(THE_CMAKE_BINARY_DIR "\${THIS_MVG_CONFIG_PATH}")
	SET(THE_MVG_CONFIG_FILE_INCLUDE_DIR "\${THIS_MVG_CONFIG_PATH}/include/mvg/mvg_config/")
ELSE(WIN32)
	# Unix install. This .cmake file will end up in /usr/share/mvg/MVGConfig.cmake :
	IF (CMAKE_MVG_USE_DEB_POSTFIXS)
		# We're building a .deb package: DESTDIR is NOT the final installation directory:
		SET(THE_MVG_SOURCE_DIR "/usr")
		SET(THE_MVG_LIBS_INCL_DIR "${THE_MVG_SOURCE_DIR}/include/mvg")
		SET(THE_CMAKE_BINARY_DIR "/usr")
		SET(THE_MVG_CONFIG_FILE_INCLUDE_DIR "/usr/include/mvg/mvg_config/")
	ELSE(CMAKE_MVG_USE_DEB_POSTFIXS)
		# Normal case: take the desired installation directory
		SET(THE_MVG_SOURCE_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MVG_LIBS_INCL_DIR "${THE_MVG_SOURCE_DIR}/include/mvg")
		SET(THE_CMAKE_BINARY_DIR "${CMAKE_INSTALL_PREFIX}")
		SET(THE_MVG_CONFIG_FILE_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include/mvg/mvg_config/")
	ENDIF(CMAKE_MVG_USE_DEB_POSTFIXS)
ENDIF(WIN32)

CONFIGURE_FILE(
	"${MVG_SOURCE_DIR}/parse-files/MVGConfig.cmake.in"  
	"${MVG_BINARY_DIR}/unix-install/MVGConfig.cmake" @ONLY IMMEDIATE )
#support for version checking when finding MVG, e.g. find_package(MVG 1.0.0 EXACT)
CONFIGURE_FILE(
	"${MVG_SOURCE_DIR}/parse-files/MVGConfig-version.cmake.in" 
	"${MVG_BINARY_DIR}/unix-install/MVGConfig-version.cmake" IMMEDIATE @ONLY)
