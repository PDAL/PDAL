#
# Draco plugin CMake configuration
#
if (NOT Draco_FOUND)
    message(FATAL_ERROR "Can't find Draco support required.")
endif()

#
# Draco Reader
#
PDAL_ADD_PLUGIN(reader_libname reader draco
    FILES
        io/DracoReader.cpp
    LINK_WITH
        ${DRACO_LIBRARY}
    INCLUDES
        ${draco_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

if (WIN32)
    target_compile_definitions(${reader_libname} PRIVATE -DNONMINMAX)
endif()

#
# Draco Writer
#

PDAL_ADD_PLUGIN(writer_libname writer draco
    FILES
        io/DracoWriter.cpp
    LINK_WITH
        ${DRACO_LIBRARY}
    INCLUDES
        ${draco_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

if (WIN32)
    target_compile_definitions(${writer_libname} PRIVATE -DNONMINMAX)
endif()

if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_io_draco_writer_test
        FILES
            test/DracoWriterTest.cpp
        LINK_WITH
            ${writer_libname}
            ${reader_libname}
            ${DRACO_LIBRARY}
        INCLUDES
            ${draco_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )

    PDAL_ADD_TEST(pdal_io_draco_reader_test
        FILES
            test/DracoReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
            ${DRACO_LIBRARY}
        INCLUDES
            ${draco_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )


if (WIN32)
    target_compile_definitions(pdal_io_draco_writer_test PRIVATE -DNONMINMAX)
    target_compile_definitions(pdal_io_draco_reader_test PRIVATE -DNONMINMAX)
endif()

endif()
