# Matlab plugin

find_package(Matlab REQUIRED)

PDAL_ADD_PLUGIN(libname writer matlab
    FILES
        io/MatlabWriter.cpp
    LINK_WITH
        ${MATLAB_LIBRARIES}
)
target_include_directories(${libname} PRIVATE
    ${MATLAB_INCLUDE_DIR})

target_compile_definitions(${libname} PRIVATE -DHAVE_MATLAB=1)
target_include_directories(${libname} PRIVATE ${MATLAB_INCLUDE_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(matlabtest
        FILES
            test/MatlabWriterTest.cpp
        LINK_WITH
            ${libname}
    )
    target_include_directories(matlabtest PRIVATE
        ${PDAL_IO_DIR}
        ${MATLAB_INCLUDE_DIR})

endif()
