#
# Faux driver CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/io/faux)

#
# Faux Reader
#
set(srcs
    FauxReader.cpp
)

set(incs
    FauxReader.hpp
)

PDAL_ADD_DRIVER(reader faux "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
