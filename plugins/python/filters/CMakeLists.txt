set(PLANG ../plang/Invocation.cpp
          ../plang/Environment.cpp
          ../plang/Redirector.cpp
          ../plang/Script.cpp)

PDAL_ADD_PLUGIN(python_libname filter python
    FILES
        ${PLANG}
        PythonFilter.cpp
    LINK_WITH
        ${PYTHON_LIBRARY}
        ${PDAL_JSONCPP_LIB_NAME}
        ${CMAKE_DL_LIBS}
    INCLUDES
        ${PYTHON_INCLUDE_DIR}
        ${PDAL_JSONCPP_INCLUDE_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_python_test
        FILES
            ../test/PythonFilterTest.cpp
        LINK_WITH
            ${python_libname} ${PYTHON_LIBRARY}
    )
endif()
