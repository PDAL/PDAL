get_property(EXISTS GLOBAL PROPERTY _PDAL_KAZHDAN_INCLUDED)
if (EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${PDAL_KAZHDAN_LIB_NAME} STATIC
    Factor.cpp
    Geometry.cpp
    MarchingCubes.cpp
)
if (UNIX)
    target_compile_options(${PDAL_KAZHDAN_LIB_NAME} PRIVATE -fPIC)
endif()
target_link_libraries(${PDAL_KAZHDAN_LIB_NAME})

set_property(GLOBAL PROPERTY _PDAL_KAZHDAN_INCLUDED TRUE)
