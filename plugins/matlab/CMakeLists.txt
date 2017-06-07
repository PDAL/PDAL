# Matlab plugin

find_package(Matlab QUIET)

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


    PDAL_ADD_TEST(mlangtest
        FILES ./test/MLangTest.cpp
        LINK_WITH ${PDAL_MLANG_LIB_NAME})
    target_include_directories(mlangtest PRIVATE
        ${ROOT_DIR} ${MATLAB_INCLUDE_DIR})
endif()
