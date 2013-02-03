link_directories( ../../bin )
include_directories (. ../../include ../..)

set( sources
    Draw.cpp
    HexGrid.cpp
    Processor.cpp
    Segment.cpp
    ../AppSupport.cpp
)

set( headers
    Draw.hpp
    Point.hpp
    Hexagon.hpp
    HexGrid.hpp
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
