#
# Derivative driver CMake configuration
#

#
# Derivative Writer
#
set(srcs
    DerivativeWriter.cpp
)

set(incs
    DerivativeWriter.hpp
)

PDAL_ADD_DRIVER(writer derivative "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
