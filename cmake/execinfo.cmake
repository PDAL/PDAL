#
# See if we have an execinfo library.
#

find_library(EXECINFO_LIBRARIES execinfo)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EXECINFO
    REQUIRED_VARS
        EXECINFO_LIBRARIES)
message("#### EXECINFO LIBRARIES = ${EXECINFO_LIBRARIES}")
if (HAVE_EXECINFO)
    message("#### Found EXECINFO = ${EXECINFO_LIBRARIES}")
    set(EXECINFO_LIBRARY ${EXECINFO_LIBRARIES})
endif()
