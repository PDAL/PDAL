cmake_minimum_required(VERSION 3.5)

if (STANDALONE)
    project(pdal-arrow)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
    include(${PDAL_CMAKE_DIR}/nlohmann.cmake)
endif(STANDALONE)

include(${PDAL_CMAKE_DIR}/arrow.cmake)

if (NOT Arrow_FOUND)
    message(FATAL_ERROR "Arrow-cpp can't be found. Can't build Arrow support")
endif()



PDAL_ADD_PLUGIN(arrow_writer_libname writer arrow
    FILES
        io/ArrowWriter.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        Arrow::arrow_shared
        Parquet::parquet_shared
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
)

PDAL_ADD_PLUGIN(arrow_reader_libname reader arrow
    FILES
        io/ArrowReader.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        Arrow::arrow_shared
        Parquet::parquet_shared
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
)

if (WIN32)
    target_compile_definitions(${arrow_writer_libname} PRIVATE NOMINMAX=1)
    target_compile_definitions(${arrow_reader_libname} PRIVATE NOMINMAX=1)
endif(WIN32)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_arrow_writer_test
        FILES
            test/ArrowWriterTest.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        Arrow::arrow_shared
        ${arrow_writer_libname}
    )

    PDAL_ADD_TEST(pdal_io_arrow_reader_test
        FILES
            test/ArrowReaderTest.cpp
    LINK_WITH
        ${PDAL_LIBRARIES}
        Arrow::arrow_shared
        ${arrow_reader_libname}
    )


if (WIN32)
    target_compile_definitions(${pdal_io_arrow_writer_test} PRIVATE NOMINMAX=1)
    target_compile_definitions(${pdal_io_arrow_reader_test} PRIVATE NOMINMAX=1)
endif(WIN32)

endif(WITH_TESTS)
