#
# Splitter filter CMake configuration
#

#
# Splitter Filter
#
set(srcs
    SplitterFilter.cpp
)

set(incs
    SplitterFilter.hpp
)

PDAL_ADD_DRIVER(filter splitter "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
