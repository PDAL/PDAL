
# Predicate Filter
#
set(srcs PredicateFilter.cpp)
set(incs PredicateFilter.hpp)

PDAL_ADD_PLUGIN(predicate_libname filter predicate
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PYTHON_LIBRARY} ${PLANG_LIB_NAME})

#
# Programmable Filter
#
set(srcs ProgrammableFilter.cpp)
set(incs ProgrammableFilter.hpp)

PDAL_ADD_PLUGIN(programmable_libname filter programmable
    FILES "${srcs}" "${incs}"
    LINK_WITH ${PYTHON_LIBRARY} ${PLANG_LIB_NAME})

if (WITH_TESTS)
    set(srcs
        ../test/PredicateFilterTest.cpp
        ../test/ProgrammableFilterTest.cpp
    )

    PDAL_ADD_TEST(pythontest
            FILES "${srcs}"
            LINK_WITH ${predicate_libname} ${programmable_libname}
                ${PLANG_LIB_NAME})
endif()
