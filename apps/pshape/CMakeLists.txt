find_package(cairo)

include_directories (. ../ ../../include ../../boost )

set( sources
    HexGrid.cpp
    Hexagon.cpp
    Path.cpp
    Processor.cpp
    Segment.cpp
    Tester.cpp
    ../AppSupport.cpp
)

set( headers
    GridInfo.hpp
    Hexagon.hpp
    HexGrid.hpp
    Mathpair.hpp
    Path.hpp
    Processor.hpp
    Segment.hpp
    ../AppSupport.hpp
)

set(PDAL_LIB_NAME
    pdal
)

set(APPS_CPP_DEPENDENCIES
    ${PDAL_LIB_NAME}
    ${CAIRO_LIBRARIES}
)

link_directories( ../../bin )
add_executable(pshape ${sources} ${headers} )
target_link_libraries(pshape ${APPS_CPP_DEPENDENCIES})
