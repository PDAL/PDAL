#
# Python plugin CMake configuration
#

set(PLANG_LIB_NAME plang)

include(${PDAL_CMAKE_DIR}/python.cmake)

add_subdirectory(plang)
add_subdirectory(filters)
