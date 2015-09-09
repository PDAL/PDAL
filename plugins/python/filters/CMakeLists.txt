
# Predicate Filter
#
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})

set(srcs PredicateFilter.cpp)
set(incs PredicateFilter.hpp)

PDAL_ADD_PLUGIN(predicate_libname filter predicate
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PDAL_PLANG_LIB_NAME})

#
# Programmable Filter
#
set(srcs ProgrammableFilter.cpp)
set(incs ProgrammableFilter.hpp)

PDAL_ADD_PLUGIN(programmable_libname filter programmable
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PDAL_PLANG_LIB_NAME})

if (WITH_TESTS)
    PDAL_ADD_TEST(python_predicate_test
            FILES ../test/PredicateFilterTest.cpp
            LINK_WITH ${predicate_libname} ${PDAL_PLANG_LIB_NAME})

    PDAL_ADD_TEST(python_programmable_test
            FILES ../test/ProgrammableFilterTest.cpp
            LINK_WITH ${programmable_libname} ${PDAL_PLANG_LIB_NAME})
endif()
