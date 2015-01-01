#
# Python plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/python)

find_package(PythonInterp QUIET)
find_package(PythonLibs QUIET 2.4)
find_package(NumPy QUIET 1.5 REQUIRED)
if(PYTHONLIBS_FOUND)
    include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
    include_directories(SYSTEM ${NUMPY_INCLUDE_DIR})

    #
    # Predicate Filter
    #
    set(srcs filters/PredicateFilter.cpp)
    set(incs filters/PredicateFilter.hpp)

    PDAL_ADD_PLUGIN(predicate_libname filter predicate
        FILES "${srcs}" "${incs}"
        LINK_WITH ${PYTHON_LIBRARY})

    #
    # Programmable Filter
    #
    set(srcs filters/ProgrammableFilter.cpp)
    set(incs filters/ProgrammableFilter.hpp)

    set(deps ${PYTHON_LIBRARY})
    PDAL_ADD_PLUGIN(programmable_libname filter programmable
        FILES "${srcs}" "${incs}"
        LINK_WITH ${PYTHON_LIBRARY})

    if (WITH_TESTS)
        include_directories(${PROJECT_SOURCE_DIR}/filters/stats)

	set(srcs
	    test/PredicateFilterTest.cpp
	    test/ProgrammableFilterTest.cpp
	)

	include_directories(${PDAL_INCLUDE_DIR}/test/unit)
	PDAL_ADD_TEST(pythontest
            FILES "${srcs}"
            LINK_WITH ${predicate_libname} ${programmable_libname})
    endif()
endif()
