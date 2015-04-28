set(srcs
    RialtoCommon.cpp
    RialtoWriter.cpp
)

set(incs
    RialtoCommon.hpp
    RialtoWriter.hpp
)

PDAL_ADD_DRIVER(writer rialto "${srcs}" "${incs}" objects)
set(PDAL_TARGET_OBJECTS ${PDAL_TARGET_OBJECTS} ${objects} PARENT_SCOPE)
