#
# Text driver CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/drivers/text)

#
# Text Writer
#
set(srcs
    TextWriter.cpp
)

set(incs
    TextWriter.hpp
)

PDAL_ADD_DRIVER(writer text "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
