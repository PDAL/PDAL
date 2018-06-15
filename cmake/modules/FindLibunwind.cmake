# Find Libunwind
# ~~~~~~~~~
# CMake module to search for LIBUNWIND
#
 
find_library(LIBUNWIND_LIBRARY unwind)
find_path(LIBUNWIND_INCLUDE_DIR libunwind.h)

find_package_handle_standard_args(LIBUNWIND REQUIRED_VARS
    LIBUNWIND_LIBRARY LIBUNWIND_INCLUDE_DIR)

if (LIBUNWIND_FOUND)
  set(LIBUNWIND_LIBRARIES ${LIBUNWIND_LIBRARY})
  set(LIBUNWIND_INCLUDE_DIRS ${LIBUNWIND_INCLUDE_DIR})
endif()
