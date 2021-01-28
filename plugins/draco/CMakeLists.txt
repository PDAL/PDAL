#
# Draco plugin CMake configuration
#
if (NOT Draco_FOUND)
    message(FATAL_ERROR "Can't find Draco support required.")
endif()

if (WIN32)
    add_definitions(-DNOMINMAX)
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

target_link_directories(${reader_libname} PRIVATE ${draco_LIBRARY_DIR})

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

target_link_directories(${writer_libname} PRIVATE ${draco_LIBRARY_DIR})


if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_io_draco_writer_test
        FILES
            test/DracoWriterTest.cpp
        LINK_WITH
            ${writer_libname}
            ${reader_libname}
        INCLUDES
            ${draco_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )
    target_link_directories(pdal_io_draco_writer_test PRIVATE ${draco_LIBRARY_DIR})

    PDAL_ADD_TEST(pdal_io_draco_reader_test
        FILES
            test/DracoReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
        INCLUDES
            ${draco_INCLUDE_DIR}
            ${NLOHMANN_INCLUDE_DIR}
    )
    target_link_directories(pdal_io_draco_reader_test PRIVATE ${draco_LIBRARY_DIR})


endif()
