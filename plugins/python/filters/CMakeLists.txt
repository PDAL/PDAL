
# Predicate Filter
#
PDAL_ADD_PLUGIN(predicate_libname filter predicate
    FILES
        PredicateFilter.cpp
    LINK_WITH ${PDAL_PLANG_LIB_NAME})
target_include_directories(${predicate_libname} PRIVATE
    ${PYTHON_INCLUDE_DIR})

#
# Programmable Filter
#

PDAL_ADD_PLUGIN(programmable_libname filter programmable
    FILES
        ProgrammableFilter.cpp
    LINK_WITH ${PDAL_PLANG_LIB_NAME})
target_include_directories(${programmable_libname} PRIVATE
    ${PYTHON_INCLUDE_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(python_predicate_test
        FILES ../test/PredicateFilterTest.cpp
        LINK_WITH ${predicate_libname} ${PDAL_PLANG_LIB_NAME})

    PDAL_ADD_TEST(python_programmable_test
        FILES ../test/ProgrammableFilterTest.cpp
        LINK_WITH ${programmable_libname} ${PDAL_PLANG_LIB_NAME})
endif()
