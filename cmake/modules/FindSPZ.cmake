#
# CMake module to search for SPZ library
#

IF(spz_INCLUDE_DIR)
    SET(spz_FIND_QUIETLY TRUE)
ENDIF()

FIND_PATH(spz_INCLUDE_DIR
    load-spz.h
    PATHS
    /usr/include
    /usr/local/include  
)

FIND_LIBRARY(spz_LIBRARY NAMES spz)

if(spz_FOUND)
  if(spz_LIBRARY)
    set(spz_LIBRARIES ${spz_LIBRARY})
  endif()
  set(spz_INCLUDE_DIRS ${spz_INCLUDE_DIR})
endif()

FIND_PACKAGE_HANDLE_STANDARD_ARGS(spz DEFAULT_MSG spz_LIBRARY spz_INCLUDE_DIR)