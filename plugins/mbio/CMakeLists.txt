cmake_minimum_required(VERSION 3.13)

if (STANDALONE)
    project(pdal-mbio)
    include(../../cmake/standalone.cmake NO_POLICY_SCOPE)
endif(STANDALONE)

#
# MBIO plugin CMake configuration (MB-System)
#

find_package(MBSystem)
if (MBSYSTEM_FOUND)
    PDAL_ADD_PLUGIN(reader_libname reader mbio
        FILES
            io/MbReader.cpp
            io/MbFormat.cpp
            io/MbError.cpp
        LINK_WITH
            ${MBSYSTEM_LIBRARY}
            ${PDAL_LIBRARIES}
        INCLUDES
            ${PDAL_IO_DIR}
            ${MBSYSTEM_INCLUDE_DIR}
    )
    target_compile_definitions(${reader_libname} PRIVATE -DHAVE_MBSYSTEM=1)

    if (WITH_TESTS)
        PDAL_ADD_TEST(mbsystemtest
            FILES
                test/MBSystemTest.cpp
            LINK_WITH
                ${reader_libname}
            INCLUDES
                ${PDAL_IO_DIR}
        )
    endif()
else()
    message(STATUS "Building without MB-Sytem support")
endif()
