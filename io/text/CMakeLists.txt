#
# Text driver CMake configuration
#

set(srcs
    TextReader.cpp
    TextWriter.cpp
)

set(incs
    TextReader.hpp
    TextWriter.hpp
)

PDAL_ADD_DRIVER(writer text "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
