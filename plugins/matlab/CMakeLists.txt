# Matlab plugin

find_package(Matlab REQUIRED)

include_directories(${MATLAB_INCLUDE_DIR})
add_definitions(-DHAVE_MATLAB=1)

set(inc io/MatlabWriter.hpp)
set(src io/MatlabWriter.cpp)

PDAL_ADD_PLUGIN(libname writer matlab
    FILES "${src}" "${inc}"
    LINK_WITH ${MATLAB_LIBRARIES}
    )

if (WITH_TESTS)
    include_directories(io)
    PDAL_ADD_TEST(matlabtest
        FILES test/MatlabWriterTest.cpp
        LINK_WITH ${libname}
        )

endif()
