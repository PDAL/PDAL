#
# See if we have an execinfo library.
#

find_library(EXECINFO_LIBRARIES execinfo)
message("#### EXECINFO = ${EXECINFO_LIBRARIES}")
if (EXECINFO_LIBRARIES)
    message("#### Found EXECINFO = ${EXECINFO_LIBRARIES}")
    set(EXECINFO_LIBRARY ${EXECINFO_LIBRARIES})
endif()
