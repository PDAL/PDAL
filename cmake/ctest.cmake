#
# CTEST support
#
if(ENABLE_CTEST)
    if (WITH_TESTS)
        message(STATUS
            "Enable CTest to support submissions of results to CDash at http://cdash.org")
        cmake_minimum_required(VERSION 2.8.0)
        #
        # Dashboard has been prepared for experiments
        # http://my.cdash.org/index.php?project=PDAL
        #
        include(CTest)
        message(STATUS
            "Enable CTest to support submissions of results to CDash at http://cdash.org - done")
        #
        # Define "make check" as alias for "make test"
        #
        ADD_CUSTOM_TARGET(check COMMAND ctest)
    else()
        message(WARNING
            "CTest support requested but WITH_TESTS option not specified to build of PDAL unit tests")
    endif()
endif()
