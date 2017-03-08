# - try to find MBSYSTEM library
#
# Cache Variables: (probably not for direct use in your scripts)
#  MBSYSTEM_INCLUDE_DIR
#  MBSYSTEM_LIBRARY
#
# Non-cache variables you might use in your CMakeLists.txt:
#  MBSYSTEM_FOUND
#  MBSYSTEM_INCLUDE_DIRS
#  MBSYSTEM_LIBRARIES
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Author:
# 2011 Philippe Crassous (ENSAM ParisTech / Institut Image) p.crassous _at_ free.fr
#
# Adapted from the Virtual Reality Peripheral Network library.
# https://github.com/rpavlik/vrpn/blob/master/README.Legal
#

set(MBSYSTEM_ROOT_DIR
	"${MBSYSTEM_ROOT_DIR}"
	CACHE
	PATH
	"Directory to search for MBSYSTEM")

find_library(MBSYSTEM_LIBRARY
	NAMES
	mbio
	PATHS
	"${MBSYSTEM_ROOT_DIR}/libs"
	/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE})

find_path(MBSYSTEM_INCLUDE_DIR
	NAMES
	mb_io.h
	PATHS
	"${MBSYSTEM_ROOT_DIR}"
	/usr/include/mbsystem
	PATH_SUFFIXES
	include)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MBSYSTEM
	DEFAULT_MSG
	MBSYSTEM_LIBRARY
	MBSYSTEM_INCLUDE_DIR)

if(MBSYSTEM_FOUND)
	set(MBSYSTEM_LIBRARIES "${MBSYSTEM_LIBRARY}")
	set(MBSYSTEM_INCLUDE_DIRS "${MBSYSTEM_INCLUDE_DIR}")
	mark_as_advanced(MBSYSTEM_ROOT_DIR)
endif()

mark_as_advanced(MBSYSTEM_INCLUDE_DIR MBSYSTEM_LIBRARY)

