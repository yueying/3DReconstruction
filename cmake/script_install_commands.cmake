# ----------------------------------------------------------------------------
#   More installation commands:
# ----------------------------------------------------------------------------
FILE(GLOB MVG_PKGCONFIG_PC_FILES "${MVG_BINARY_DIR}/pkgconfig/mvg_*.pc")

IF(EXISTS "${MVG_BINARY_DIR}/pkgconfig/mvg_base.pc" AND NOT IS_DEBIAN_DBG_PKG)
	INSTALL(
		FILES ${MVG_PKGCONFIG_PC_FILES}
		DESTINATION ${libmvg_dev_INSTALL_PREFIX}${CMAKE_INSTALL_LIBDIR}/pkgconfig )
ENDIF(EXISTS "${MVG_BINARY_DIR}/pkgconfig/mvg_base.pc" AND NOT IS_DEBIAN_DBG_PKG)

# CMake will look for MVGConfig.cmake at: /usr/share|lib/mvg
IF(WIN32)
	INSTALL(FILES "${MVG_BINARY_DIR}/unix-install/MVGConfig.cmake" DESTINATION ./ )
	INSTALL(FILES "${MVG_BINARY_DIR}/unix-install/MVGConfig-version.cmake" DESTINATION ./ )
ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MVG_BINARY_DIR}/unix-install/MVGConfig.cmake" DESTINATION ${libmvg_dev_INSTALL_PREFIX}share/mvg )
		INSTALL(FILES "${MVG_BINARY_DIR}/unix-install/MVGConfig-version.cmake" DESTINATION ${libmvg_dev_INSTALL_PREFIX}share/mvg )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# Docs, examples and the rest of files:
IF(WIN32)
	INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/doc" DESTINATION ./ )

	IF (PACKAGE_INCLUDES_SOURCES)
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/cmake" DESTINATION ./ )
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/3rdparty" DESTINATION ./ )
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/samples"  DESTINATION ./ COMPONENT Examples )
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/scripts" DESTINATION ./  )
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/tests" DESTINATION ./  )
	ENDIF (PACKAGE_INCLUDES_SOURCES)
		
	INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/parse-files" DESTINATION ./ )
	INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share" DESTINATION ./  )

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
	file(TO_CMAKE_PATH "$ENV{MVG_EXTRA_DLLS_TO_INSTALL}" MVG_EXTRA_DLLS_TO_INSTALL)
	IF (NOT "${MVG_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")
		if (EXISTS "${MVG_EXTRA_DLLS_TO_INSTALL}")
			FILE(STRINGS "${MVG_EXTRA_DLLS_TO_INSTALL}" MVG_EXTRA_DLLS_TO_INSTALL_FILES)
			FOREACH(XFIL ${MVG_EXTRA_DLLS_TO_INSTALL_FILES})
				file(TO_CMAKE_PATH "${XFIL}" XFIL2)
				INSTALL(FILES "${XFIL2}" DESTINATION bin)
			ENDFOREACH(XFIL)
		endif (EXISTS "${MVG_EXTRA_DLLS_TO_INSTALL}")
	ENDIF(NOT "${MVG_EXTRA_DLLS_TO_INSTALL}" STREQUAL "")

	# My own debug DLLs:
	FILE(GLOB_RECURSE EXTRA_DLLS "${MVG_BINARY_DIR}/bin/Debug/*.dll")
	FOREACH(F ${EXTRA_DLLS})
		INSTALL(FILES "${F}" DESTINATION bin)
	ENDFOREACH(F)
	FILE(GLOB_RECURSE EXTRA_LIBS "${MVG_BINARY_DIR}/lib/Debug/*.lib")
	FOREACH(F ${EXTRA_LIBS})
		INSTALL(FILES "${F}" DESTINATION lib)
	ENDFOREACH(F)

ELSE(WIN32)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/doc/html" DESTINATION ${mvg_doc_INSTALL_PREFIX}share/doc/mvg_doc/  )
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/samples" DESTINATION ${mvg_doc_INSTALL_PREFIX}share/doc/mvg_doc/  )
		INSTALL(FILES "${MVG_SOURCE_DIR}/doc/mvg_example1.tar.gz" DESTINATION ${mvg_doc_INSTALL_PREFIX}share/doc/mvg_doc/ )
		IF(EXISTS "${MVG_SOURCE_DIR}/doc/mvg_book.ps.gz")
			INSTALL(FILES "${MVG_SOURCE_DIR}/doc/mvg_book.ps.gz" DESTINATION ${mvg_doc_INSTALL_PREFIX}share/doc/mvg_doc/ )
		ENDIF(EXISTS "${MVG_SOURCE_DIR}/doc/mvg_book.ps.gz")

		# applications config files
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share/applications" DESTINATION ${mvg_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share/mvg" DESTINATION ${mvg_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share/pixmaps" DESTINATION ${mvg_apps_INSTALL_PREFIX}share)
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share/appdata" DESTINATION ${mvg_apps_INSTALL_PREFIX}share)

	 	# Mime types go to the mvg_core package
		INSTALL(DIRECTORY "${MVG_SOURCE_DIR}/share/mime" DESTINATION ${mvg_apps_INSTALL_PREFIX}share )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF(WIN32)

# The headers of all the MVG libs:
# (in win32 the /libs/* tree is install entirely, not only the headers):
IF (PACKAGE_INCLUDES_SOURCES)
	IF (UNIX AND NOT IS_DEBIAN_DBG_PKG)
		FOREACH(_LIB ${ALL_MVG_LIBS})
			STRING(REGEX REPLACE "mvg_(.*)" "\\1" _LIB ${_LIB})
			SET(SRC_DIR "${MVG_SOURCE_DIR}/libs/${_LIB}/include/")
			IF (EXISTS "${SRC_DIR}")  # This is mainly to avoid a crash with "mvg_core", which is a "virtual" MVG module.
				INSTALL(DIRECTORY "${SRC_DIR}" DESTINATION ${libmvg_dev_INSTALL_PREFIX}include/mvg/${_LIB}/include/  )
			ENDIF (EXISTS "${SRC_DIR}")
		ENDFOREACH(_LIB)
	ENDIF(UNIX AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)


# Config-dependent headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (NOT IS_DEBIAN_DBG_PKG)
		INSTALL(FILES "${MVG_CONFIG_FILE_INCLUDE_DIR}/mvg/config.h" DESTINATION "${libmvg_dev_INSTALL_PREFIX}include/mvg/mvg_config/mvg/" )
		INSTALL(FILES "${MVG_CONFIG_FILE_INCLUDE_DIR}/mvg/version.h" DESTINATION "${libmvg_dev_INSTALL_PREFIX}include/mvg/mvg_config/mvg/" )
	ENDIF(NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)

# If using embedded version, install embedded version as part of mvg_base's headers:
IF (PACKAGE_INCLUDES_SOURCES)
	IF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
		IF(WIN32)
			# Eigen headers must end up in /Program Files/MVG-X.Y.Z/libs/base/...
			SET(MVG_INSTALL_EIGEN_PREFIX "libs/base/include/")
		ELSE(WIN32)
			# Eigen headers must end up in /usr/...
			SET(MVG_INSTALL_EIGEN_PREFIX "${libmvg_dev_INSTALL_PREFIX}include/mvg/base/include/")
		ENDIF(WIN32)

		INSTALL(
			DIRECTORY "${MVG_SOURCE_DIR}/3drparty/eigen3/Eigen"
			DESTINATION "${MVG_INSTALL_EIGEN_PREFIX}" )
		INSTALL(
			DIRECTORY "${MVG_SOURCE_DIR}/3drparty/eigen3/unsupported"
			DESTINATION "${MVG_INSTALL_EIGEN_PREFIX}" )
	ENDIF (EIGEN_USE_EMBEDDED_VERSION AND NOT IS_DEBIAN_DBG_PKG)
ENDIF (PACKAGE_INCLUDES_SOURCES)
