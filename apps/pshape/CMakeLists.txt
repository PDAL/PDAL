include_directories (.)

set( sources
    processor.cpp
    HexGrid.cpp
)

set( headers
    Point.hpp
    Hexagon.hpp
    HexGrid.hpp
)

add_executable(pshape ${sources} ${headers} )
