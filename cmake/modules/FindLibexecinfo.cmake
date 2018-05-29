# Find Libexecinfo
# ~~~~~~~~~
# CMake module to search for LIBEXECINFO
#

find_path(LIBEXECINFO_INCLUDE_DIR execinfo.h)

#
# On Apple backtrace is in a system library.
#
if (NOT APPLE)
  find_library(LIBEXECINFO_LIBRARY execinfo)
  find_package_handle_standard_args(LIBEXECINFO
    REQUIRED_VARS
      LIBEXECINFO_LIBRARY LIBEXECINFO_INCLUDE_DIR)
else()
  find_package_handle_standard_args(LIBEXECINFO
    REQUIRED_VARS
      LIBEXECINFO_INCLUDE_DIR)
endif()

if (LIBEXECINFO_FOUND)
  if (LIBEXECINFO_LIBRARY)
    set(LIBEXECINFO_LIBRARIES ${LIBEXECINFO_LIBRARY})
  endif()
  set(LIBEXECINFO_INCLUDE_DIRS ${LIBEXECINFO_INCLUDE_DIR})
endif()
