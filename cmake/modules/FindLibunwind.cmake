# Find Libunwind
# ~~~~~~~~~
# CMake module to search for LIBUNWIND
#

find_path(LIBUNWIND_INCLUDE_DIR libunwind.h)

if(NOT APPLE)
  find_library(LIBUNWIND_LIBRARY unwind)
  find_package_handle_standard_args(
    LIBUNWIND REQUIRED_VARS LIBUNWIND_LIBRARY LIBUNWIND_INCLUDE_DIR)
else()
  find_package_handle_standard_args(LIBUNWIND
                                    REQUIRED_VARS LIBUNWIND_INCLUDE_DIR)
endif()

if(LIBUNWIND_FOUND)
  if(LIBUNWIND_LIBRARY)
    set(LIBUNWIND_LIBRARIES ${LIBUNWIND_LIBRARY})
  endif()
  set(LIBUNWIND_INCLUDE_DIRS ${LIBUNWIND_INCLUDE_DIR})
endif()
