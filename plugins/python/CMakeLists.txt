#
# Python plugin CMake configuration
#

add_subdirectory(filters)
if (WITH_TESTS)
    include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
    PDAL_ADD_TEST(plangtest FILES ./test/PLangTest.cpp
        LINK_WITH ${PDAL_PLANG_LIB_NAME})
endif()
