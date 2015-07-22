#
# NULL writer CMake configuration
#

set(srcs
    NullWriter.cpp
)

set(incs
    NullWriter.hpp
)

PDAL_ADD_DRIVER(writer null "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
