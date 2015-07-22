# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------
FILE(GLOB FBLIB_PKGCONFIG_PC_FILES "${FBLIB_BINARY_DIR}/pkgconfig/fblib_*.pc")

IF(EXISTS "${FBLIB_BINARY_DIR}/pkgconfig/fblib_base.pc" AND NOT IS_DEBIAN_DBG_PKG)
	INSTALL(
		FILES ${FBLIB_PKGCONFIG_PC_FILES}
		DESTINATION ${libfblib_dev_INSTALL_PREFIX}${CMAKE_INSTALL_LIBDIR}/pkgconfig )
ENDIF(EXISTS "${FBLIB_BINARY_DIR}/pkgconfig/fblib_base.pc" AND NOT IS_DEBIAN_DBG_PKG)

# CMake will look for FBLIBConfig.cmake at: /usr/share|lib/fblib
IF(WIN32)
	INSTALL(FILES "${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig.cmake" DESTINATION ./ )
	INSTALL(FILES "${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig-version.cmake" DESTINATION ./ )
ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig.cmake" DESTINATION ${libfblib_dev_INSTALL_PREFIX}share/fblib )
		INSTALL(FILES "${FBLIB_BINARY_DIR}/unix-install/FBLIBConfig-version.cmake" DESTINATION ${libfblib_dev_INSTALL_PREFIX}share/fblib )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# Docs, examples and the rest of files:
IF(WIN32)
	INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/doc" DESTINATION ./ )

	IF (PACKAGE_INCLUDES_SOURCES)
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/cmake" DESTINATION ./ )
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/3rdparty" DESTINATION ./ )
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/samples"  DESTINATION ./ COMPONENT Examples )
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/scripts" DESTINATION ./  )
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/tests" DESTINATION ./  )
	ENDIF (PACKAGE_INCLUDES_SOURCES)
		
	INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/parse-files" DESTINATION ./ )
	INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share" DESTINATION ./  )

	# Smart determination of the dependencies DLLs so they are also copied when installing:
	# ---------------------------------------------------------------------------------------

	# OpenCV:
	IF (EXISTS "${OpenCV_DIR}/bin/Release")
		FILE(GLOB_RECURSE EXTRA_DLLS "${OpenCV_DIR}/bin/*.dll") # This includes debug & release DLLs
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${OpenCV_DIR}/bin/Release")

	# ffmpeg:
	IF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")
		FILE(GLOB_RECURSE EXTRA_DLLS "${FFMPEG_WIN32_ROOT_DIR}/bin/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)
	ENDIF (EXISTS "${FFMPEG_WIN32_ROOT_DIR}/bin")

	# Extra optional DLLs to be installed in the "bin" folder:
	file(TO_CMAKE_PATH "$ENV{FBLIB_EXTRA_DLLS_TO_INSTALL}" FBLIB_EXTRA_DLLS_TO_INSTALL)
	IF (NOT "${FBLIB_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${FBLIB_EXTRA_DLLS_TO_INSTALL}")
			FILE(STRINGS "${FBLIB_EXTRA_DLLS_TO_INSTALL}" FBLIB_EXTRA_DLLS_TO_INSTALL_FILES)
			FOREACH(XFIL ${FBLIB_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				INSTALL(FILES "${XFIL2}" DESTINATION bin)
			ENDFOREACH(XFIL)
		endif (EXISTS "${FBLIB_EXTRA_DLLS_TO_INSTALL}")
	ENDIF(NOT "${FBLIB_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")

	# My own debug DLLs:
	FILE(GLOB_RECURSE EXTRA_DLLS "${FBLIB_BINARY_DIR}/bin/Debug/*.dll")
	FOREACH(F ${EXTRA_DLLS})
		INSTALL(FILES "${F}" DESTINATION bin)
	ENDFOREACH(F)
	FILE(GLOB_RECURSE EXTRA_LIBS "${FBLIB_BINARY_DIR}/lib/Debug/*.lib")
	FOREACH(F ${EXTRA_LIBS})
		INSTALL(FILES "${F}" DESTINATION lib)
	ENDFOREACH(F)

ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/doc/html" DESTINATION ${fblib_doc_INSTALL_PREFIX}share/doc/fblib_doc/  )
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/samples" DESTINATION ${fblib_doc_INSTALL_PREFIX}share/doc/fblib_doc/  )
		INSTALL(FILES "${FBLIB_SOURCE_DIR}/doc/fblib_example1.tar.gz" DESTINATION ${fblib_doc_INSTALL_PREFIX}share/doc/fblib_doc/ )
		IF(EXISTS "${FBLIB_SOURCE_DIR}/doc/fblib_book.ps.gz")
			INSTALL(FILES "${FBLIB_SOURCE_DIR}/doc/fblib_book.ps.gz" DESTINATION ${fblib_doc_INSTALL_PREFIX}share/doc/fblib_doc/ )
		ENDIF(EXISTS "${FBLIB_SOURCE_DIR}/doc/fblib_book.ps.gz")

		# applications config files
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share/applications" DESTINATION ${fblib_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share/fblib" DESTINATION ${fblib_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share/pixmaps" DESTINATION ${fblib_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share/appdata" DESTINATION ${fblib_apps_INSTALL_PREFIX}share)

	 	# Mime types go to the fblib_core package
		INSTALL(DIRECTORY "${FBLIB_SOURCE_DIR}/share/mime" DESTINATION ${fblib_apps_INSTALL_PREFIX}share )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# The headers of all the FBLIB libs:
# (in win32 the /libs/* tree is install entirely, not only the headers):
IF (PACKAGE_INCLUDES_SOURCES)
	IF (UNIX AND NOT IS_DEBIAN_DBG_PKG)
		FOREACH(_LIB ${ALL_FBLIB_LIBS})
			STRING(REGEX REPLACE "fblib_(.*)" "\\1" _LIB ${_LIB})
			SET(SRC_DIR "${FBLIB_SOURCE_DIR}/libs/${_LIB}/include/")
			IF (EXISTS "${SRC_DIR}")  # This is mainly to avoid a crash with "fblib_core", which is a "virtual" FBLIB module.
				INSTALL(DIRECTORY "${SRC_DIR}" DESTINATION ${libfblib_dev_INSTALL_PREFIX}include/fblib/${_LIB}/include/  )
			ENDIF (EXISTS "${SRC_DIR}")
		ENDFOREACH(_LIB)
	ENDIF(UNIX AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)


# Config-dependent headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${FBLIB_CONFIG_FILE_INCLUDE_DIR}/fblib/config.h" DESTINATION "${libfblib_dev_INSTALL_PREFIX}include/fblib/fblib_config/fblib/" )
		INSTALL(FILES "${FBLIB_CONFIG_FILE_INCLUDE_DIR}/fblib/version.h" DESTINATION "${libfblib_dev_INSTALL_PREFIX}include/fblib/fblib_config/fblib/" )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)

# If using embedded version, install embedded version as part of fblib_base's headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
		IF(WIN32)
			# Eigen headers must end up in /Program Files/FBLIB-X.Y.Z/libs/base/...
			SET(FBLIB_INSTALL_EIGEN_PREFIX "libs/base/include/")
		ELSE(WIN32)
			# Eigen headers must end up in /usr/...
			SET(FBLIB_INSTALL_EIGEN_PREFIX "${libfblib_dev_INSTALL_PREFIX}include/fblib/base/include/")
		ENDIF(WIN32)

		INSTALL(
			DIRECTORY "${FBLIB_SOURCE_DIR}/3drparty/eigen3/Eigen"
			DESTINATION "${FBLIB_INSTALL_EIGEN_PREFIX}" )
		INSTALL(
			DIRECTORY "${FBLIB_SOURCE_DIR}/3drparty/eigen3/unsupported"
			DESTINATION "${FBLIB_INSTALL_EIGEN_PREFIX}" )
	ENDIF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)
