#
# See if we have an execinfo library.
#

find_library(EXECINFO_LIBRARIES execinfo)
if (EXECINFO_LIBRARIES)
    set(EXECINFO_LIBRARY ${EXECINFO_LIBRARIES})
endif()
