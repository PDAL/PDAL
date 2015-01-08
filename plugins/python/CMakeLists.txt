#
# Python plugin CMake configuration
#

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

PDAL_ADD_PLUGIN(programmable_libname filter programmable
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PYTHON_LIBRARY})

if (WITH_TESTS)
	set(srcs
	    test/PredicateFilterTest.cpp
	    test/ProgrammableFilterTest.cpp
	)

	PDAL_ADD_TEST(pythontest
            FILES "${srcs}"
            LINK_WITH ${predicate_libname} ${programmable_libname})
endif()
