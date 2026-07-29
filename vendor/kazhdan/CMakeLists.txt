get_property(EXISTS GLOBAL PROPERTY _PDAL_KAZHDAN_INCLUDED)
if (EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_KAZHDAN_LIB_NAME} STATIC ${PDAL_LIB_TYPE}
    Factor.cpp
    Geometry.cpp
    MarchingCubes.cpp
)
if (UNIX)
    set_property(TARGET ${PDAL_KAZHDAN_LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_compile_options(${PDAL_KAZHDAN_LIB_NAME} PRIVATE -Wno-maybe-uninitialized)
endif()
target_link_libraries(${PDAL_KAZHDAN_LIB_NAME})

set_property(GLOBAL PROPERTY _PDAL_KAZHDAN_INCLUDED TRUE)
