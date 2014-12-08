#
# Python plugin CMake configuration
#

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/plugins/python)

find_package(PythonInterp)
find_package(PythonLibs 2.4)
find_package(NumPy 1.5 REQUIRED)
if(PYTHONLIBS_FOUND)
    message(STATUS "Building with Python support")
    message(STATUS "Found Python: ${PYTHON_LIBRARY}")

    include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})
    include_directories(SYSTEM ${NUMPY_INCLUDE_DIR})

    #
    # Predicate Filter
    #
    set(srcs
	filters/PredicateFilter.cpp
    )

    set(incs
	filters/PredicateFilter.hpp
    )

    set(deps ${PYTHON_LIBRARY})
    PDAL_ADD_PLUGIN(predicate_filter_lib_name filter predicate "${srcs}" "${incs}" "${deps}")

    #
    # Programmable Filter
    #
    set(srcs
	filters/ProgrammableFilter.cpp
    )

    set(incs
	filters/ProgrammableFilter.hpp
    )

    set(deps ${PYTHON_LIBRARY})
    PDAL_ADD_PLUGIN(programmable_filter_lib_name filter programmable "${srcs}" "${incs}" "${deps}")

    if (WITH_TESTS)
        include_directories(${PROJECT_SOURCE_DIR}/filters/stats)

        #
	# Python tests
	#
	set(srcs
	    test/PredicateFilterTest.cpp
	    test/ProgrammableFilterTest.cpp
	)

	set(deps
	    ${predicate_filter_lib_name}
	    ${programmable_filter_lib_name}
	)

	include_directories(${PDAL_INCLUDE_DIR}/test/unit)
	PDAL_ADD_TEST(pythontest "${srcs}" "${deps}")
    endif()
else()
    message(STATUS "Building with Python support")
endif()
