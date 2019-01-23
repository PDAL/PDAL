#
# FBX plugin CMake configuration
#

set(FBX_INCLUDE_DIR
    "FBX_INCLUDE_DIR-NOTFOUND" CACHE PATH
    "Include directory for FBX header")
set(FBX_LIBRARY
    "FBX_LIBRARY-NOTFOUND" CACHE STRING
    "Full path to FBX library")
set_package_properties(Fbx PROPERTIES TYPE OPTIONAL)
if (EXISTS "${FBX_INCLUDE_DIR}/fbxsdk.h" AND
    EXISTS ${FBX_LIBRARY})
    set(FBX_FOUND true)
else()
    set(FBX_FOUND false)
endif()

if (APPLE)
    set(CORE_FOUNDATION "-framework CoreFoundation")
endif()

if (FBX_FOUND)
    PDAL_ADD_PLUGIN(writer_libname writer fbx
        FILES
            io/FbxWriter.cpp
        LINK_WITH
            ${FBX_LIBRARY}
            ${CORE_FOUNDATION}
        INCLUDES
            ${PDAL_IO_DIR}
            ${FBX_INCLUDE_DIR}
    )

    if (WITH_TESTS)
        PDAL_ADD_TEST(fbxtest
            FILES
                test/FbxWriterTest.cpp
            LINK_WITH
                ${writer_libname}
            INCLUDES
                ${PDAL_IO_DIR}
        )
    endif()
else()
    message(STATUS "Building without FBX support")
endif()
