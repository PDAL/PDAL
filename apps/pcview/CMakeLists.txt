#
# ./apps/pcview CMake configuration file for OpenGL-based viewer
#


include_directories(
    .
    ../../include
    ${FREEGLUT_INCLUDE_DIRS})


set(PCVIEW pcview)


SET(PDAL_PCVIEW_CPP
    ArcBall.cpp
    ArcBallMath.cpp
    Commands.cpp
    Controller.cpp
    Engine.cpp
    main.cpp
    )

SET(PDAL_PCVIEW_HPP
    ArcBall.hpp
    ArcBallMath.hpp
    Commands.hpp
    Controller.hpp
    Engine.hpp
    glext.h
    )
 

#------------------------------------------------------------------------------
# source groups in IDE
#------------------------------------------------------------------------------
 
source_group("CMake Files" FILES CMakeLists.txt)
source_group("Header Files" FILES  
    Commands.hpp
    Controller.hpp
    Engine.hpp
)
source_group("Header Files\\ArcBall" FILES 
    ArcBall.hpp
    ArcBallMath.hpp
)
source_group("Header Files\\config" FILES 
    glext.h
)
source_group("Source Files" FILES 
    Commands.cpp
    Controller.cpp
    Engine.cpp
    main.cpp
)
source_group("Source Files\\ArcBall" FILES 
    ArcBall.cpp
    ArcBallMath.cpp
)


#------------------------------------------------------------------------------
# Configure build targets
#------------------------------------------------------------------------------

set(APPS_CPP_DEPENDENCIES
    ${PDAL_LIB_NAME}
    ${LIBLAS_LIBRARY}
    ${TIFF_LIBRARY}
    ${GEOTIFF_LIBRARY}
    ${GDAL_LIBRARY}
    ${SPATIALINDEX_LIBRARY}
    ${LASZIP_LIBRARY}
    ${Boost_LIBRARIES}
    ${FREEGLUT_LIBRARY})

link_directories(${Boost_LIBRARY_DIRS})

add_executable(${PCVIEW} ${PDAL_PCVIEW_CPP} ${PDAL_PCVIEW_HPP})
target_link_libraries(${PCVIEW} ${APPS_CPP_DEPENDENCIES} )
