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
	PDAL_ADD_TEST(pdal_filters_predicate_test
        FILES test/PredicateFilterTest.cpp
        LINK_WITH ${predicate_libname})
	PDAL_ADD_TEST(pdal_filters_programmable_test
        FILES test/ProgrammableFilterTest.cpp
        LINK_WITH ${programmable_libname})
endif()
