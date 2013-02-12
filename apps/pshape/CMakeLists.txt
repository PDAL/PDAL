link_directories( ../../bin )

find_package(X11)
find_package(cairo)


include_directories (. ../../include ${X11_X11_INCLUDE_PATH})

set( sources
    Draw.cpp
    HexGrid.cpp
    Hexagon.cpp
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


set(APPS_CPP_DEPENDENCIES
    ${PDAL_LIB_NAME}
    ${X11_X11_LIB}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY}
    ${LASZIP_LIBRARY}
    ${Boost_LIBRARIES}
    ${FREEGLUT_LIBRARY})


set( libs
    ${CAIRO_LIBRARIES}
    ${X11_X11_LIB}
)

add_executable(pshape ${sources} ${headers} )
target_link_libraries(pshape ${libs}  ${APPS_CPP_DEPENDENCIES})
