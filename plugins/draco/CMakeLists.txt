#
# Draco plugin CMake configuration
#

#
# We use pkg_check_modules because the draco Cmake config is broken
find_package(PkgConfig REQUIRED)
pkg_check_modules(DRACO REQUIRED draco>1.4.0)

if (WIN32)
    add_definitions(-DNOMINMAX)
endif()

if (UNIX)
    set(PDAL_DRACO_COMPILE_OPTIONS "-Wno-deprecated-copy;-Wno-sign-compare;-Wno-ignored-qualifiers")
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

if (UNIX)
    target_compile_options(${reader_libname} PRIVATE
                           ${PDAL_DRACO_COMPILE_OPTIONS})
endif()


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
if (UNIX)
    target_compile_options(${writer_libname} PRIVATE
                           ${PDAL_DRACO_COMPILE_OPTIONS})
endif()


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
    if (UNIX)
        target_compile_options(pdal_io_draco_writer_test PRIVATE
                               ${PDAL_DRACO_COMPILE_OPTIONS})
    endif()

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
    if (UNIX)
        target_compile_options(pdal_io_draco_reader_test PRIVATE
                               ${PDAL_DRACO_COMPILE_OPTIONS})
    endif()


endif()
