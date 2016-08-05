#
# Text driver CMake configuration
#

set(srcs
    PtsReader.cpp
)

set(incs
    PtsReader.hpp
)

PDAL_ADD_DRIVER(reader pts "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
