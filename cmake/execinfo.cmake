#
# See if we have an execinfo library.
#

find_library(EXECINFO_LIBRARIES execinfo)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EXECINFO
    REQUIRED_VARS
        EXECINFO_LIBRARIES)
if (HAVE_EXECINFO)
    set(EXECINFO_LIBRARY ${EXECINFO_LIBRARIES})
endif()
