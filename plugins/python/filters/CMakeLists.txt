set(PLANG ../plang/Invocation.cpp
          ../plang/Environment.cpp
          ../plang/Redirector.cpp
          ../plang/Script.cpp)

PDAL_ADD_PLUGIN(python_libname filter python
    FILES
        ${PLANG}
        PythonFilter.cpp
    LINK_WITH
        ${CMAKE_DL_LIBS}
    INCLUDES
        ${NLOHMANN_INCLUDE_DIR}
    SYSTEM_INCLUDES
        ${PYTHON_ALL_INCLUDE_DIRS}
)
target_compile_definitions(${python_libname} PRIVATE
    PDAL_PYTHON_LIBRARY="${PYTHON_LIBRARY}")
target_link_options(${python_libname} PRIVATE "-Wl,-undefined" "-Wl,dynamic_lookup")

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_filters_python_test
        FILES
            ../test/PythonFilterTest.cpp
        LINK_WITH
            ${python_libname} ${PYTHON_LIBRARY}
        SYSTEM_INCLUDES
            ${PYTHON_ALL_INCLUDE_DIRS}
    )
endif()
