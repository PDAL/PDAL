#
# Faux driver CMake configuration
#

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
