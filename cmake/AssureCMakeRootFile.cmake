# Assure the calling CMakeLists.txt file is included from the correct ROOT CMakeLists.txt file,
# and not from scratch due to a user mistake.
# We do this by checking the existence of a predefined variable which will only exist if the real
# root file is being processed.
#
#  Usage:  INCLUDE(AssureCMakeRootFile)
#

IF(NOT FBLIB_SOURCE_DIR)
	MESSAGE(FATAL_ERROR "ERROR: Do not use this directory as 'source directory' in CMake, but the ROOT directory of the FBLIB source tree.")
ENDIF(NOT FBLIB_SOURCE_DIR)


