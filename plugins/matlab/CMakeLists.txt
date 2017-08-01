# Matlab plugin

include(${PDAL_CMAKE_DIR}/matlab.cmake)

#set(CMAKE_SKIP_RPATH true)
#if (LINUX)

#endif()

PDAL_ADD_PLUGIN(libname_writer writer matlab
    FILES
        ./filters/Script.cpp
        io/MatlabWriter.cpp
    LINK_WITH
        ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
)
target_include_directories(${libname_writer} PRIVATE
    ${MATLAB_INCLUDE_DIR})

PDAL_ADD_PLUGIN(libname_reader reader matlab
    FILES
        ./filters/Script.cpp
        io/MatlabReader.cpp
    LINK_WITH
        ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
)
target_include_directories(${libname_reader} PRIVATE
    ${MATLAB_INCLUDE_DIR})


PDAL_ADD_PLUGIN(matlab_libname filter matlab
    FILES
        ./filters/Environment.cpp
        ./filters/Script.cpp
        ./filters/MatlabFilter.cpp
    LINK_WITH
        ${MATLAB_ENG_LIBRARY} ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
    )
target_include_directories(${matlab_libname} PRIVATE
    ${MATLAB_INCLUDE_DIR}
    ${PDAL_JSONCPP_INCLUDE_DIR} )

if (WITH_TESTS)
    PDAL_ADD_TEST(pdal_io_matlab_writer_test
        FILES
            test/MatlabWriterTest.cpp
        LINK_WITH
           ${libname_writer}
    )
    target_include_directories(pdal_io_matlab_writer_test PRIVATE
        ${PDAL_IO_DIR}
        ${MATLAB_INCLUDE_DIR})
    set_target_properties(pdal_io_matlab_writer_test
        PROPERTIES
        INSTALL_RPATH "${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES}:${MATLAB_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH true
    )


    PDAL_ADD_TEST(pdal_io_matlab_reader_test
        FILES
            test/MatlabReaderTest.cpp
        LINK_WITH
           ${libname_reader}
    )
    target_include_directories(pdal_io_matlab_reader_test PRIVATE
        ${PDAL_IO_DIR}
        ${MATLAB_INCLUDE_DIR})
    set_target_properties(pdal_io_matlab_reader_test
        PROPERTIES
        INSTALL_RPATH "${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES}:${MATLAB_LIB_DIR}"
        BUILD_WITH_INSTALL_RPATH true
    )

    PDAL_ADD_TEST(pdal_filters_matlab_test
        FILES
            ./filters/Script.cpp
            ./filters/Environment.cpp
            ./test/MatlabFilterTest.cpp
        LINK_WITH
            ${MATLAB_ENG_LIBRARY}
            ${MATLAB_MX_LIBRARY}
            ${MATLAB_MAT_LIBRARY}
    )
    set_target_properties(pdal_filters_matlab_test
        PROPERTIES
        INSTALL_RPATH "${CMAKE_CXX_IMPLICIT_LINK_DIRECTORIES}:${MATLAB_LIB_DIR}:${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
        BUILD_WITH_INSTALL_RPATH true
    )
]]
    target_include_directories(pdal_filters_matlab_test PRIVATE
        ${ROOT_DIR}
        ${MATLAB_INCLUDE_DIR}
        ${PDAL_JSONCPP_INCLUDE_DIR} )
endif()

