link_directories( ../../bin )
include_directories (. ../../include ../..)

set( sources
    Draw.cpp
    Hexagon.cpp
    HexGrid.cpp
    Processor.cpp
    Segment.cpp
    ../AppSupport.cpp
)

set( headers
    Draw.hpp
    Hexagon.hpp
    HexGrid.hpp
    Mathpair.hpp
    Segment.hpp
    ../AppSupport.hpp
)

set( libs
    libpdal.so
    libcairo.so
    libX11.so
)

add_executable(pshape ${sources} ${headers} )
target_link_libraries(pshape ${libs})
