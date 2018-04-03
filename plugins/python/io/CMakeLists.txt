PDAL_ADD_PLUGIN(python_libname reader numpy
    FILES
        NumpyReader.cpp
    )
target_include_directories(${python_libname} PRIVATE
    ${PYTHON_INCLUDE_DIR} ${PDAL_JSONCPP_INCLUDE_DIR})

target_link_libraries(${python_libname} PUBLIC
    ${PYTHON_LIBRARY} ${CMAKE_DL_LIBS} PRIVATE ${PDAL_PLANG_LIB_NAME})

if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_io_numpy_test
        FILES ../test/NumpyReaderTest.cpp
        LINK_WITH ${python_libname} )
    target_link_libraries(pdal_io_numpy_test PUBLIC
        ${PYTHON_LIBRARY} ${CMAKE_DL_LIBS}  PRIVATE  ${PDAL_PLANG_LIB_NAME})

endif()
