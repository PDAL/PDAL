
PDAL_ADD_PLUGIN(numpy_reader reader numpy
    FILES
        NumpyReader.cpp
        ../plang/Environment.cpp
        ../plang/Redirector.cpp
    LINK_WITH
        ${PYTHON_LIBRARY}
        ${CMAKE_DL_LIBS}
    INCLUDES
        ${PYTHON_INCLUDE_DIR}
    )

# Install headers so Python extension
# can use them later
install(DIRECTORY .
    DESTINATION  include/pdal/io
    FILES_MATCHING PATTERN "*.hpp"
    PATTERN "CMakeFiles" EXCLUDE
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_numpy_test
        FILES
            ../test/NumpyReaderTest.cpp
        LINK_WITH
            ${numpy_reader} ${PYTHON_LIBRARY})
endif()
