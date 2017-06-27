# Matlab plugin

include(${PDAL_CMAKE_DIR}/matlab.cmake)

set(MLANG ./filters/Environment.cpp
          ./filters/Script.cpp)

PDAL_ADD_PLUGIN(libname writer matlab
    FILES
        ./filters/Script.cpp
        io/MatlabWriter.cpp
    LINK_WITH
        ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
)
target_include_directories(${libname} PRIVATE
    ${MATLAB_INCLUDE_DIR})

target_compile_definitions(${libname} PRIVATE -DHAVE_MATLAB=1)
target_include_directories(${libname} PRIVATE ${MATLAB_INCLUDE_DIR})

PDAL_ADD_PLUGIN(libname_reader reader matlab
    FILES
        ./filters/Script.cpp
        io/MatlabReader.cpp
    LINK_WITH
        ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
)
target_include_directories(${libname_reader} PRIVATE
    ${MATLAB_INCLUDE_DIR})

target_compile_definitions(${libname_reader} PRIVATE -DHAVE_MATLAB=1)
target_include_directories(${libname_reader} PRIVATE ${MATLAB_INCLUDE_DIR})




PDAL_ADD_PLUGIN(matlab_libname filter matlab
    FILES
        ./filters/Environment.cpp
        ./filters/Script.cpp
        ./filters/MatlabFilter.cpp
    LINK_WITH
        ${MATLAB_ENG_LIBRARY} ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
    )
target_include_directories(${matlab_libname} PRIVATE
    ${MATLAB_INCLUDE_DIR} )

if (WITH_TESTS)
    PDAL_ADD_TEST(matlabtest
        FILES
            test/MatlabWriterTest.cpp
        LINK_WITH
           ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY} ${libname}
    )
    target_include_directories(matlabtest PRIVATE
        ${PDAL_IO_DIR}
        ${MATLAB_INCLUDE_DIR})


    PDAL_ADD_TEST(pdal_filters_matlab_test
        FILES
            ./filters/Script.cpp
            ./filters/Environment.cpp
            ./test/MatlabFilterTest.cpp
        LINK_WITH ${MATLAB_ENG_LIBRARY} ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY}
)
    target_include_directories(pdal_filters_matlab_test PRIVATE
        ${ROOT_DIR} ${MATLAB_INCLUDE_DIR})
endif()

