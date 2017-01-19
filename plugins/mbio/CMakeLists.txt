#
# MBIO plugin CMake configuration (MB-System)
#

find_package(MBSYSTEM)
if (MBSYSTEM_FOUND)
    PDAL_ADD_PLUGIN(reader_libname reader mbio
        FILES
            io/MBSystemReader.cpp
            io/mbutil.cpp
        LINK_WITH ${MBSYSTEM_LIBRARY})
    target_include_directories(${reader_libname} PRIVATE
        ${PDAL_IO_DIR}
        ${MBSYSTEM_INCLUDE_DIR})
    target_compile_definitions(${reader_libname} PRIVATE -DHAVE_MBSYSTEM=1)

    if (WITH_TESTS)
        PDAL_ADD_TEST(mbsystemtest
            FILES test/MBSystemTest.cpp
            LINK_WITH ${reader_libname})
        target_include_directories(mbsystemtest PRIVATE
            ${PDAL_IO_DIR})
    endif()
else()
    message(STATUS "Building without MB-Sytem support")
endif()
