include_directories (.)

set( sources
    Processor.cpp
    HexGrid.cpp
    Segment.cpp
)

set( headers
    Point.hpp
    Hexagon.hpp
    HexGrid.hpp
    Segment.hpp
)

add_executable(pshape ${sources} ${headers} )
