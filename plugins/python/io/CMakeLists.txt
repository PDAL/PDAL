
PDAL_ADD_PLUGIN(numpy_reader reader numpy
    FILES
        NumpyReader.cpp
        ../plang/Environment.cpp
        ../plang/Redirector.cpp
    )

target_include_directories(${numpy_reader} PRIVATE
    ${PYTHON_INCLUDE_DIR})

target_link_libraries(${numpy_reader} PUBLIC
    ${PYTHON_LIBRARY})

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
            ../plang/Environment.cpp
            ../plang/Redirector.cpp
        LINK_WITH ${numpy_reader} )
    target_link_libraries(pdal_io_numpy_test PUBLIC
        ${PYTHON_LIBRARY})
endif()
