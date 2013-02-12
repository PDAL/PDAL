find_package(cairo)


include_directories (. ../ ../../include )

set( sources
    Draw.cpp
    HexBin.cpp
    HexGrid.cpp
    Hexagon.cpp
    Processor.cpp
    Segment.cpp
    ../AppSupport.cpp
)

set( headers
    Draw.hpp
    Hexagon.hpp
    HexBin.hpp
    HexGrid.hpp
    Mathpair.hpp
    Segment.hpp
    ../AppSupport.hpp
)


set(APPS_CPP_DEPENDENCIES
    ${PDAL_LIB_NAME}
    ${CAIRO_LIBRARIES}
)

add_executable(pshape ${sources} ${headers} )
target_link_libraries(pshape ${APPS_CPP_DEPENDENCIES})
