#
# Draco plugin CMake configuration
#
include (${PDAL_CMAKE_DIR}/draco.cmake)

if (NOT PDAL_HAVE_DRACO)
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
        ${DRACO_LIBRARIES}
    INCLUDES
        ${DRACO_INCLUDE_DIRS}
        ${NLOHMANN_INCLUDE_DIR}
)

target_link_libraries(${reader_libname}  PRIVATE ${DRACO_LIBRARY})
target_link_directories(${reader_libname} PRIVATE ${DRACO_LIBRARY_DIRS})

#
# Draco Writer
#

PDAL_ADD_PLUGIN(writer_libname writer draco
    FILES
        io/DracoWriter.cpp
    LINK_WITH
        ${DRACO_LIBRARIES}
    INCLUDES
        ${DRACO_INCLUDE_DIRS}
        ${NLOHMANN_INCLUDE_DIR}
)

target_link_directories(${writer_libname} PRIVATE ${DRACO_LIBRARY_DIRS})


if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_io_draco_writer_test
        FILES
            test/DracoWriterTest.cpp
        LINK_WITH
            ${writer_libname}
            ${reader_libname}
        INCLUDES
            ${DRACO_INCLUDE_DIRS}
            ${NLOHMANN_INCLUDE_DIR}
    )
    target_link_directories(pdal_io_draco_writer_test PRIVATE ${DRACO_LIBRARY_DIRS})
    target_link_libraries(pdal_io_draco_writer_test  PRIVATE ${DRACO_LIBRARIES})

    PDAL_ADD_TEST(pdal_io_draco_reader_test
        FILES
            test/DracoReaderTest.cpp
        LINK_WITH
            ${reader_libname}
            ${writer_libname}
        INCLUDES
            ${DRACO_INCLUDE_DIRS}
            ${NLOHMANN_INCLUDE_DIR}
    )
    target_link_directories(pdal_io_draco_reader_test PRIVATE ${DRACO_LIBRARY_DIRS})
    target_link_libraries(pdal_io_draco_reader_test  PRIVATE ${DRACO_LIBRARIES})


endif()
