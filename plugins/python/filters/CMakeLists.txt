PDAL_ADD_PLUGIN(python_libname filter python
    FILES
        ../plang/Invocation.cpp
        ../plang/Environment.cpp
        ../plang/Redirector.cpp
        ../plang/Script.cpp
        PythonFilter.cpp
    LINK_WITH ${PDAL_PLANG_LIB_NAME})
target_include_directories(${python_libname} PRIVATE
    ${PYTHON_INCLUDE_DIR} ${PDAL_JSONCPP_INCLUDE_DIR})

target_link_libraries(${python_libname} PUBLIC
    ${PYTHON_LIBRARY})

if (WITH_TESTS)

    PDAL_ADD_TEST(pdal_filters_python_test
        FILES ../test/PythonFilterTest.cpp
        LINK_WITH ${programmable_libname} ${PDAL_PLANG_LIB_NAME})
    target_link_libraries(pdal_filters_python_test PUBLIC
        ${PYTHON_LIBRARY})

endif()
