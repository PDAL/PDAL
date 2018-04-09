
PDAL_ADD_PLUGIN(numpy_reader reader numpy
    FILES
        NumpyReader.cpp
        ../plang/Environment.cpp
        ../plang/Redirector.cpp
    LINK_WITH
        ${PYTHON_LIBRARY}
        ${CMAKE_DL_LIBS}
    )

target_include_directories(${numpy_reader} PRIVATE
    ${PYTHON_INCLUDE_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_numpy_test
        FILES
            ../test/NumpyReaderTest.cpp
        LINK_WITH
            ${numpy_reader})
endif()
