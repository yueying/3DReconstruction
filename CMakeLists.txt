# ----------------------------------------------------------------------------
# Root CMake file for the MVG libraries and applications
#
#  Run with "cmake ." at the root directory to build the makefiles for
#   the MVG C++ library, the samples, and the applications. Some scripts
#   for generating the documentation, etc. are also updated.
#
#  2014-2015, fengbing <fengbing123@gmail.com>
#
#  NOTE: CMake can be obtained at http://www.cmake.org/
# ----------------------------------------------------------------------------

PROJECT(MVG)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

# -------------------------
#        Setup CMake
# -------------------------
# Allow empty endif() and such with CMake 2.4.
SET(CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS 1)

if(COMMAND cmake_policy)
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW)
	if(POLICY CMP0053)
		cmake_policy(SET CMP0053 OLD) # Simplify variable reference and escape sequence evaluation. 
	endif()
	if(POLICY CMP0026)
		cmake_policy(SET CMP0026 OLD) # Allow using the LOCATION target property.
	endif()
	if(POLICY CMP0037)
		cmake_policy(SET CMP0037 OLD)  # Allow defining target "test"
	endif()
	if(POLICY CMP0043)
		cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.
	endif()
	
endif()
# -------------------------


include(cmake/script_version_number.cmake REQUIRED)	#  Loads MVG version number

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake") # Directory for CMake scripts

# The root directory for all MVG libraries/modules:
SET(MVG_LIBS_ROOT "${CMAKE_SOURCE_DIR}/libs" CACHE INTERNAL "")  # This emulates global vars

SET(MVG_SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR})
SET(MVG_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
SET(MVG_BINARY_DIR  ${PROJECT_BINARY_DIR})

# Windows default: lib. Debian pkgs: set from outside rules script
IF(NOT DEFINED CMAKE_INSTALL_LIBDIR)
	SET(CMAKE_INSTALL_LIBDIR "lib")
ENDIF(NOT DEFINED CMAKE_INSTALL_LIBDIR)

# ---------------------------------------------------------------------------
# Create the minimum list of libraries required by an application that
#  uses the MVG C++ library. This must be passed to "TARGET_LINK_LIBRARIES"
# NOTE: For the compilers MSVC* & Borland most of the libraries are
#        automatically included through "#pragma"'s ;-)
# ---------------------------------------------------------------------------
SET(MVG_LINKER_LIBS "")

# ----- Useful macros ------
include(cmake/UtilsMacros.cmake REQUIRED)
include(cmake/FilterFileLists.cmake REQUIRED)
include(cmake/MVGPCHSupport.cmake REQUIRED)
include(cmake/DeclareMVGLib.cmake REQUIRED)
include(cmake/DeclareAppDependencies.cmake REQUIRED)
#INCLUDE(cmake/Installation.cmake REQUIRED)
# --------------------------

include(cmake/script_detect_unix_arch.cmake REQUIRED) # Detect machine architecture, on UNIX
include(cmake/script_detect_gcc.cmake REQUIRED)       # Detect GNU GCC version
include(cmake/script_detect_clang.cmake REQUIRED)     # Detect clang compiler version


# ------------------------------------------
# We want libraries to be named "mvg_XXX010.lib" 
#  and "mvg_XXX010d.lib" in all compilers:
# ------------------------------------------
set(CMAKE_DEBUG_POSTFIX  "d")
IF(MSVC)
	set(MVG_LIB_PREFIX "")
ENDIF(MSVC)

# In case of Makefiles if the user does not setup CMAKE_BUILD_TYPE, assume it's Release:
if (${CMAKE_GENERATOR} MATCHES ".*Makefiles")
    if("${CMAKE_BUILD_TYPE}" STREQUAL "")
        set(CMAKE_BUILD_TYPE Release)
    endif()
endif()

#include(cmake/script_select_app_to_build.cmake REQUIRED)		# Build (or not) some apps:

# ----------------------------------------------------------------------------
# 		CHECK FOR SYSTEM LIBRARIES, OPTIONS, ETC..
# ----------------------------------------------------------------------------

# Detect wordsize:
# -------------------------------------------
IF(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MVG_WORD_SIZE 64)
ELSE(CMAKE_SIZEOF_VOID_P EQUAL 8)  # Size in bytes!
	SET(CMAKE_MVG_WORD_SIZE 32)
ENDIF(CMAKE_SIZEOF_VOID_P EQUAL 8)

# Build static or dynamic libs?
# ===================================================
# Default: static libraries:
SET(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries (.dll/.so) instead of static ones (.lib/.a)")
IF(BUILD_SHARED_LIBS)
	SET(CMAKE_MVG_BUILD_SHARED_LIB "#define MVG_BUILT_AS_DLL")
	SET(CMAKE_MVG_BUILD_SHARED_LIB_ONOFF 1)
ELSE(BUILD_SHARED_LIBS)
	SET(CMAKE_MVG_BUILD_SHARED_LIB "/* #define MVG_BUILT_AS_DLL */")
	SET(CMAKE_MVG_BUILD_SHARED_LIB_ONOFF 0)
ENDIF(BUILD_SHARED_LIBS)

# Only for Unix: use pkg-config to find libraries
# ===================================================
INCLUDE(FindPkgConfig OPTIONAL)
# "QUIET" was introduced in 2.8.2 for PKG_CHECK_MODULES
if (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION} VERSION_GREATER 2.8.1)
   set(_QUIET QUIET)
endif()


# Group projects in "folders"
# ===================================================
IF (CMAKE_VERSION VERSION_GREATER 2.8.0)
	SET(ENABLE_SOLUTION_FOLDERS ON CACHE BOOL "Group projects under virtual folders of the compiler IDE (e.g. VisualStudio)")
ENDIF (CMAKE_VERSION VERSION_GREATER 2.8.0)

if (ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif (ENABLE_SOLUTION_FOLDERS)


# GCC only:
# ===================================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	# Enable libstdc++ parallel mode?
	SET(MVG_ENABLE_LIBSTD_PARALLEL_MODE OFF CACHE BOOL "Enable parallel mode in libstdc++ (requires GCC 4.2.2+)")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# Enable precompiled headers:
# ===================================================
#IF(MSVC)
#	SET(DEFAULT_VAL_PRECOMP_HDR ON)
#ELSE(MSVC)
#	SET(DEFAULT_VAL_PRECOMP_HDR OFF)
#ENDIF(MSVC)

#SET(MVG_ENABLE_PRECOMPILED_HDRS ${DEFAULT_VAL_PRECOMP_HDR} CACHE BOOL "Enable precompiled headers (thru 'mvg/LIB.h')")
SET(MVG_ENABLE_PRECOMPILED_HDRS OFF CACHE BOOL "Enable precompiled headers (thru 'mvg/LIB.h')")
#MARK_AS_ADVANCED(MVG_ENABLE_PRECOMPILED_HDRS)

# Eigen library plugins:
# ===================================================
include(cmake/script_eigen.cmake REQUIRED)

# MVG_TRY_START/END blocks
# ===================================================
SET(MVG_HAS_STACKED_EXCEPTIONS ON CACHE BOOL "Enable MVG_TRY_START/END blocks (disable it for speed up).")

# Optional MVG embedded profiler:
# ===================================================
SET(MVG_ENABLE_EMBEDDED_GLOBAL_PROFILER OFF CACHE BOOL "Enable MVG embedded profiler.")

# ASSERT_ blocks
# ===================================================
SET(MVG_HAS_ASSERT ON CACHE BOOL "Enable ASSERT_ statements (disable it for speed up).")

#   "Clasic" function & headers detection:
include(cmake/script_detect_functions_headers.cmake REQUIRED)

# MSVC only: 
# ===================================================
IF(MSVC)
	# Enable Parallel compilation?
	SET(COMPILE_IN_PARALLEL ON CACHE BOOL "Enable parallel compilation in Visual Studio")
ENDIF(MSVC)

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# Must be invoked *before* other embedded projects so MVG's target "uninstall" exists first
# ----------------------------------------------------------------------------
CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/parse-files/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(uninstall PROPERTIES FOLDER "CMakeTargets")
endif(ENABLE_SOLUTION_FOLDERS)

# ----------------------------------------------------------------------------
# Other sub-scripts:
# ----------------------------------------------------------------------------
include(cmake/script_opencv.cmake REQUIRED)      # Check for the OpenCV libraries (via pkg-config, CMake, with different options)
include(cmake/script_ffmpeg.cmake REQUIRED)      # Check for ffmpeg C libraries: libavcodec, libavutil, libavformat, libswscale
include(cmake/script_zlib.cmake REQUIRED)        # Check for zlib
include(cmake/script_jpeg.cmake REQUIRED)        # Check for jpeg
include(cmake/script_SIMD.cmake REQUIRED)        # SSE2/SSE3/... optimization options
include(cmake/script_gtest.cmake REQUIRED)       # Unit testing lib
include(cmake/script_gl_glut.cmake REQUIRED)     # Check for the GL,GLUT libraries 
include(cmake/script_daisy.cmake REQUIRED)       # daisy lib
include(cmake/script_fast.cmake REQUIRED)        # fast lib
include(cmake/script_flann.cmake REQUIRED)       # flann lib
include(cmake/script_ssba.cmake REQUIRED)        # ssba lib
# ---------------------------------------------------------------------------
#			OPTIONS
#The options for the user when using "cmakesetup" or "ccmake":
# ---------------------------------------------------------------------------
OPTION(MVG_ALWAYS_CHECKS_DEBUG "Additional checks even in Release" "OFF")
OPTION(MVG_ALWAYS_CHECKS_DEBUG_MATRICES "Additional checks even in Release (Only in matrix classes)" "OFF")

# ---------------------------------------------------------------------------
# Create the minimum list of libraries required by an application that
#  uses the MVG C++ library. This must be passed to "TARGET_LINK_LIBRARIES"
# NOTE: For the compilers MSVC* & Borland most of the libraries are
#        automatically included through "#pragma"'s ;-)
# ---------------------------------------------------------------------------
IF(NOT MSVC)
	# In Linux, link against librt
	# Jerome Monceaux : 2011/03/08
	# exclude rt from apple osx linking
	IF(UNIX AND NOT APPLE)
		APPEND_MVG_LIBS(rt)
	ENDIF(UNIX AND NOT APPLE)

	# In Windows, link against ole32 & uuid & oleaut32, etc...
	IF(MINGW)
		APPEND_MVG_LIBS(ole32 uuid oleaut32 gdi32 winmm setupapi ws2_32)
	ENDIF(MINGW)
ENDIF(NOT MSVC)


SET(CMAKE_MVG_HAS_GZ_STREAMS ${CMAKE_MVG_HAS_ZLIB})   # We have gz-streams is we have zlib:

include(cmake/script_png.cmake REQUIRED)  # support png image

# Save the basic list of libraries (without MVG-libs):
# -------------------------------------------------------------------------------------
SET(MVGLIB_LINKER_LIBS ${MVG_LINKER_LIBS} ${MVG_FFMPEG_LIBS_TO_LINK})
IF($ENV{VERBOSE})
	MESSAGE(STATUS "MVGLIB_LINKER_LIBS: ${MVGLIB_LINKER_LIBS}")
ENDIF($ENV{VERBOSE})

# If we are building .so libs, don't link every program to all the libs:
IF(BUILD_SHARED_LIBS)
	SET(MVG_LINKER_LIBS "")
ENDIF(BUILD_SHARED_LIBS)

# Save the list of libraries according to the current debug/release build (gcc only)
# -------------------------------------------------------------------------------------
SET(MVG_LINKER_LIBS_RELorDEB ${MVG_LINKER_LIBS})

IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(MVG_LINKER_LIBS_POSTFIX "${MVG_DEBUG_POSTFIX}")
ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(MVG_LINKER_LIBS_POSTFIX "")
ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

# Include Asian fonts in utils::CMVGCanvas ?
SET( MVG_HAS_ASIAN_FONTS ON CACHE BOOL "Enable Asian fonts in utils::CMVGCanvas (increases library size).")

include(cmake/script_declare_defines.cmake REQUIRED)  # Transform the variables MVG_XXX="ON/OFF" to CMAKE_MVG_XXX="1/0"

include(cmake/script_create_config_h.cmake REQUIRED)   # Build config.h
include(cmake/script_create_version_h.cmake REQUIRED)  # Build version.h

# ---------------------------------------------------------------------------
# The C++ include & link directories:
# ---------------------------------------------------------------------------
INCLUDE_DIRECTORIES(".")# 首先包含当前面临

LINK_DIRECTORIES("${CMAKE_BINARY_DIR}/lib")

# Add user supplied extra options (optimization, etc...)
SET(USER_EXTRA_CPP_FLAGS "" CACHE STRING "Put extra compiler options here if desired")

# ----------------------------------------------------------------------------
# 			Set the maximum level of warnings:
# ----------------------------------------------------------------------------
SET( MVG_EXTRA_WARNINGS OFF CACHE BOOL "Enable extra warnings apart from -Wall")
MARK_AS_ADVANCED(MVG_EXTRA_WARNINGS)

# Should be set to true for development
SET( MVG_WARNINGS_ARE_ERRORS OFF CACHE BOOL "Treat warnings as errors")
MARK_AS_ADVANCED(MVG_WARNINGS_ARE_ERRORS)

# Whole program optimization?
SET( MVG_WHOLE_PROGRAM_OPTIMIZATION OFF CACHE BOOL "Flags for whole program optimization.")
MARK_AS_ADVANCED(MVG_WHOLE_PROGRAM_OPTIMIZATION)

# Even more warnings for different compilers
IF(MVG_EXTRA_WARNINGS)
   #TODO - Add options for MSVC
   IF (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wextra") # -Wold-style-cast  -Woverloaded-virtual
   ELSEIF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
	ENDIF()
ENDIF(MVG_EXTRA_WARNINGS)

# Enable profiling?
SET(MVG_ENABLE_PROFILING OFF CACHE BOOL "Enable profiling (add -g -pg in GCC/CLANG, /PROFILE in Visual C++)")

IF(MSVC)
	SET(EXTRA_CPP_FLAGS "/W3 /D _CRT_SECURE_NO_DEPRECATE /D _CRT_NONSTDC_NO_DEPRECATE")
	
	# "In earlier versions of Visual C++, the compiler used several discrete
	#   heaps, and each had a finite limit. Currently, the compiler dynamically
	#   grows the heaps as necessary up to a total heap size limit, and requires
	#   a fixed-size buffer only to construct precompiled headers. Consequently,
	#   the /Zm compiler option is rarely necessary."
 	# http://msdn.microsoft.com/en-us/library/bdscwf1c.aspx
	# 
	#if(MSVC_VERSION LESS 1700) # <=VS 2012 (from my experience, in VS2012 this isn't needed).
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /Zm1000")
	#endif(MSVC_VERSION LESS 1700)

	# Whole program optimization
	IF(MVG_WHOLE_PROGRAM_OPTIMIZATION)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /GL")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LTCG ")
		#SET(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /LTCG ")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /LTCG ")
		#SET(CMAKE_MODULE_LINKER_FLAGS_RELEASE "${CMAKE_MODULE_LINKER_FLAGS_RELEASE} /LTCG ")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /LTCG ")
	ENDIF(MVG_WHOLE_PROGRAM_OPTIMIZATION)

	# Remove unreferenced functions: function level linking
	SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /Gy")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W3 ")

	# SSE2 for MSVC?
	IF (CMAKE_MVG_HAS_SSE2 AND NOT CMAKE_MVG_WORD_SIZE EQUAL 64)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /arch:SSE2")
	ENDIF(CMAKE_MVG_HAS_SSE2 AND NOT CMAKE_MVG_WORD_SIZE EQUAL 64)

	# SSE3: any flag needed? 

	# Fast math:
	#SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} /fp:fast")
	
	# Profiling?
	IF(MVG_ENABLE_PROFILING)
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /PROFILE ")
		SET(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /PROFILE ")
	ENDIF(MVG_ENABLE_PROFILING)
ENDIF(MSVC)

# GNU GCC options ================================
IF(CMAKE_COMPILER_IS_GNUCXX)
	# Wall & pedantic?
	IF(APPLE)
		SET(MVG_BUILD_GCC_PEDANTIC_DEFAULT "OFF")
	ELSE(APPLE)
		SET(MVG_BUILD_GCC_PEDANTIC_DEFAULT "ON")


    
	ENDIF(APPLE)

	SET( MVG_BUILD_GCC_PEDANTIC ${MVG_BUILD_GCC_PEDANTIC_DEFAULT} CACHE BOOL "Enable pedantic error detection (with GCC only)")
	MARK_AS_ADVANCED(MVG_BUILD_GCC_PEDANTIC)

	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CONFIGURE_CFLAGS} -Wall -Wno-long-long -Wno-variadic-macros")
  IF(NOT APPLE)
    # This causes the option "-Wnorite-strings" to be set on gcc-4.9 on OS X
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings")
  ENDIF(NOT APPLE)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CONFIGURE_CFLAGS}")

	# Use "modern" C99 ! ;-)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")


	IF(MVG_BUILD_GCC_PEDANTIC)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
 		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	ENDIF(MVG_BUILD_GCC_PEDANTIC)

	# Enable C++11 features in GCC 4.7+
	IF (NOT CMAKE_MVG_GCC_VERSION LESS 47)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	ENDIF (NOT CMAKE_MVG_GCC_VERSION LESS 47)

	# Parallel stdlibc++?
	IF(MVG_ENABLE_LIBSTD_PARALLEL_MODE)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -D_GLIBCXX_PARALLEL -fopenmp")
	ENDIF(MVG_ENABLE_LIBSTD_PARALLEL_MODE)
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# CLang options ================================
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
	SET( MVG_BUILD_CLANG_PEDANTIC OFF CACHE BOOL "Enable pedantic error detection (clang)")
	MARK_AS_ADVANCED(MVG_BUILD_CLANG_PEDANTIC)

	# High level of warnings.
	# no-unused-private-field: clang seems to complain in templates without reason.
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CMAKE_CONFIGURE_CFLAGS} -Wall -Wno-unused-private-field")
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${CMAKE_CONFIGURE_CFLAGS}")

	# Use "modern" C99 ! ;-)
 	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")

	# Use modern C++11
 	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

	IF(NOT APPLE) # any reason for not using it in all platforms?
	# Use the libstdc++ lib vs. libc++, to avoid some build errors in MacOS
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++")
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -stdlib=libstdc++")
		SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -stdlib=libstdc++")
	ENDIF(NOT APPLE)

	# This is to fix the compilation of C++ templates
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdelayed-template-parsing")

endif()

# Shared options between GCC and CLANG:
# ======================================
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX)
	IF(MVG_WARNINGS_ARE_ERRORS)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	ENDIF(MVG_WARNINGS_ARE_ERRORS)

	# We need pthread's
	IF (UNIX AND NOT APPLE)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	ENDIF(UNIX AND NOT APPLE)

	IF(CMAKE_BUILD_TYPE MATCHES "Debug")
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -g")
		ADD_DEFINITIONS( -D_DEBUG)
		ADD_DEFINITIONS( -DDEBUG)
	ENDIF(CMAKE_BUILD_TYPE MATCHES "Debug")

	# Profiling?
	IF(MVG_ENABLE_PROFILING)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -pg -g")
	ELSEIF(MVG_ENABLE_PROFILING)
		# Remove unreferenced functions: function level linking
        # Remove unreferenced functions: function level linking
        if(NOT APPLE)
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffunction-sections")
        endif(NOT APPLE)
	ENDIF(MVG_ENABLE_PROFILING)

	# Whole program optimization
	IF(MVG_WHOLE_PROGRAM_OPTIMIZATION)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --combine")
		SET(MVG_EXES_CXX_FLAGS "${MVG_EXES_CXX_FLAGS} -fwhole-program --combine")
	ENDIF(MVG_WHOLE_PROGRAM_OPTIMIZATION)


	# "-mtune=native" generates code optimized for the detected current processor.
	if (WIN32 OR "${CMAKE_MVG_ARCH}" MATCHES ".*86" OR "${CMAKE_MVG_ARCH}" MATCHES "amd64")
		set(opt_native_def ON)
	endif (WIN32 OR "${CMAKE_MVG_ARCH}" MATCHES ".*86" OR "${CMAKE_MVG_ARCH}" MATCHES "amd64")

	SET( MVG_OPTIMIZE_NATIVE ${opt_native_def} CACHE BOOL "GCC/clang optimizations for current processor (-mtune=native). Requires GCC 4.2+")

	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")  # Not in debug!
		SET(MVG_OPTIMIZE_NATIVE OFF)
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

	IF(CMAKE_MVG_USE_DEB_POSTFIXS OR CMAKE_MVG_IS_RPM_PACKAGE)
		# If we're building a Debian package, just DO NOT use -mtune=native
		SET(EXTRA_CPP_FLAGS ${CFLAGS})
	ELSE(CMAKE_MVG_USE_DEB_POSTFIXS OR CMAKE_MVG_IS_RPM_PACKAGE)
		IF(MVG_OPTIMIZE_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
			SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -O3 -mtune=native ")
		ENDIF(MVG_OPTIMIZE_NATIVE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
	ENDIF(CMAKE_MVG_USE_DEB_POSTFIXS OR CMAKE_MVG_IS_RPM_PACKAGE)

	# "-ffast-math"
	SET( MVG_OPTIMIZE_FFAST-MATH OFF CACHE BOOL "GCC/clang optimization for floating math (-ffast-math).")
	IF(MVG_OPTIMIZE_FFAST-MATH)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -ffast-math")
	ENDIF(MVG_OPTIMIZE_FFAST-MATH)

	# SSE2?
	IF (CMAKE_MVG_HAS_SSE2)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse2 -funroll-loops")
		IF (CMAKE_COMPILER_IS_GNUCXX)
			SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -mfpmath=sse")
		ENDIF(CMAKE_COMPILER_IS_GNUCXX)
	ENDIF(CMAKE_MVG_HAS_SSE2)

	# SSE3?
	IF (CMAKE_MVG_HAS_SSE3)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse3 -mssse3")
	ENDIF(CMAKE_MVG_HAS_SSE3)

	# SSE4*?
	IF (CMAKE_MVG_HAS_SSE4_1)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4.1")
	ENDIF(CMAKE_MVG_HAS_SSE4_1)
	IF (CMAKE_MVG_HAS_SSE4_2)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4.2")
	ENDIF(CMAKE_MVG_HAS_SSE4_2)
	IF (CMAKE_MVG_HAS_SSE4_A)
		SET(EXTRA_CPP_FLAGS "${EXTRA_CPP_FLAGS} -msse4a")
	ENDIF(CMAKE_MVG_HAS_SSE4_A)

endif ()

# Add user supplied extra options (optimization, etc...)
# ==========================================================
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_CPP_FLAGS} ${USER_EXTRA_CPP_FLAGS}")


# Some tricks for MSVC:
if(MSVC)
	STRING(REGEX REPLACE "/EHsc" "/EHa" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /GS-")

	if (COMPILE_IN_PARALLEL)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")  # Multi-CPU parallel compilation (Suggested by  robert.schattschneide)
	endif (COMPILE_IN_PARALLEL)

	# For MSVC to avoid the C1128 error about too large object files:
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")

endif(MSVC)

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT APPLE)
	# --as-needed: Avoid unnecesary libraries (.so dependencies):
	#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--as-needed")
	#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--as-needed")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT APPLE)


IF($ENV{VERBOSE})
	MESSAGE(STATUS "Final CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_EXE_LINKER_FLAGS: ${CMAKE_EXE_LINKER_FLAGS}")
	MESSAGE(STATUS "Final CMAKE_SHARED_LINKER_FLAGS: ${CMAKE_SHARED_LINKER_FLAGS}")
	MESSAGE(STATUS "Final MVGLIB_LINKER_LIBS: ${MVGLIB_LINKER_LIBS}")
ENDIF($ENV{VERBOSE})

# Save libs and executables in the same place
SET( LIBRARY_OUTPUT_PATH ${MVG_BINARY_DIR}/lib CACHE PATH "Output directory for libraries" )
SET( EXECUTABLE_OUTPUT_PATH ${MVG_BINARY_DIR}/bin CACHE PATH "Output directory for applications" )

#--------------------------------------------------------------
# If we are building the final step of the Debian package,
#  save each library files in the corresponding directories:
#--------------------------------------------------------------
IF(CMAKE_MVG_USE_DEB_POSTFIXS)
	# Values when building a Debian package ---------------
	MESSAGE(STATUS "** mvg cmake: Using Debian post-fix for install directories **")
	IF(CMAKE_BUILD_TYPE STREQUAL "Debug")
		# Debug packages
		SET(libmvg_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libmvg_dbg/usr/")
	ELSE(CMAKE_BUILD_TYPE STREQUAL "Debug")
		# Normal packages
		SET(libmvg_dev_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/libmvg_dev/usr/")
	ENDIF(CMAKE_BUILD_TYPE STREQUAL "Debug")

	SET(mvg_apps_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mvg_apps/usr/")
	SET(mvg_doc_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}/mvg_doc/usr/")
	SET(mvg_pkgconfig_INSTALL_PREFIX "/usr")	# Values when building a Debian package
ELSE(CMAKE_MVG_USE_DEB_POSTFIXS)
	# Values under normal conditions -----------------------
	SET(libmvg_dev_INSTALL_PREFIX "")
	SET(mvg_apps_INSTALL_PREFIX "")
	SET(mvg_doc_INSTALL_PREFIX "")
	SET(mvg_pkgconfig_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}") 	# Values under normal conditions
ENDIF(CMAKE_MVG_USE_DEB_POSTFIXS)

# Remove /STACK:10000000 set by CMake. This value for stack size
# is very high, limiting the number of threads we can spawn.
# Default value used by Windows is 1MB which is good enough.
# By: Sahn Lam, @ CMake mailing list
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
STRING(REGEX REPLACE "/STACK:[0-9]+" "/STACK:1000000" CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")


# This will become a list with all libraries to be built, and their
#  dependencies stored in "mvg_${name}_LIB_DEPS"
SET(ALL_MVG_LIBS "" CACHE INTERNAL "")  # This emulates global vars

# ----------------------------------------------------------------------------
#      					PROCESS SUBDIRECTORIES:
# ----------------------------------------------------------------------------
add_subdirectory(3rdparty)   # The third-party libraries
add_subdirectory(libs)        # The MVG C++ libraries:

SET( BUILD_APPLICATIONS ON CACHE BOOL "If you only want the MVG libraries, disable this.")
IF(BUILD_APPLICATIONS)
	add_subdirectory(apps)    # The applications:
ENDIF(BUILD_APPLICATIONS)

# Documentation targets (must be AFTER "apps" because it uses the aux program "mvg_perfdata2html")
add_subdirectory(doc)

#   UNIT TESTS:
# ----------------------------------------------------------------------------
SET( BUILD_TESTING ON CACHE BOOL "Build MVG tests")
IF( BUILD_TESTING)
	# Try using libgtest (Google testing library) from the system, if available
	# Note: In gtest 1..0 things changed and there's no prebuilt system lib anymore.
	# TODO: Is it possible to compile here the /usr/src/gtest/ sources??
	IF(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" AND APPLE)
		add_definitions(-DGTEST_USE_OWN_TR1_TUPLE=1)
	ENDIF()
	#add_subdirectory(${CMAKE_MVG_GTEST_SRC_DIR})  # Build gtest from sources
	add_subdirectory(tests)  # Build my tests
ENDIF( BUILD_TESTING)

include(cmake/script_create_MVGConfig.cmake REQUIRED)  # Generate the MVGConfig.cmake & configure files

# Prepare CPack params for building binary packages (has to be after the apps/)
#include(cmake/script_setup_cpack.cmake REQUIRED)			

# ----------------------------------------------------------------------------
#  Hide some variables to the user, just show the important variables:
# ----------------------------------------------------------------------------
MARK_AS_ADVANCED(FORCE
	CMAKE_BACKWARDS_COMPATIBILITY
)

#-----------------------------------
# The examples
# *Note*: This must be AFTER the generation of the MVGConfig.cmake files
#-----------------------------------
add_definitions(-DMVG_OPENCV_SRC_DIR="${MVG_OPENCV_SRC_DIR}")
SET(MVG_DIR ${MVG_BINARY_DIR})
include(cmake/DefineExamples.cmake REQUIRED)

SET(IS_DEBIAN_DBG_PKG 0)
IF (CMAKE_MVG_USE_DEB_POSTFIXS AND CMAKE_BUILD_TYPE STREQUAL "Debug")
	SET(IS_DEBIAN_DBG_PKG 1)
ENDIF (CMAKE_MVG_USE_DEB_POSTFIXS AND CMAKE_BUILD_TYPE STREQUAL "Debug")

# Build list of files to install, packages, etc.
include(cmake/script_install_commands.cmake REQUIRED)  

# Summary
include(cmake/script_show_final_summary.cmake REQUIRED)

