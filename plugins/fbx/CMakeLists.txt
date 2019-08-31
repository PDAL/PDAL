#
# FBX plugin CMake configuration
#
find_package(Fbx 2016 REQUIRED)



if (APPLE)
    set(CORE_FOUNDATION "-framework CoreFoundation")
endif()

if (FBX_FOUND)
    PDAL_ADD_PLUGIN(writer_libname writer fbx
        FILES
            io/FbxWriter.cpp
        LINK_WITH
            ${FBX_LIBRARIES}
            ${CORE_FOUNDATION}
        INCLUDES
            ${PDAL_IO_DIR}
            ${FBX_INCLUDE_DIRS}
    )

    if (WITH_TESTS)
        PDAL_ADD_TEST(fbxtest
            FILES
                test/FbxWriterTest.cpp
            LINK_WITH
                ${writer_libname}
            INCLUDES
                ${PDAL_IO_DIRS}
        )
    endif()
else()
    message(STATUS "Building without FBX support")
endif()
