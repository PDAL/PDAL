
IF (NOT WIN32)
project(perf_tests)

include_directories(../../include)

# Performance tests.  Each test is currently a simple executable
add_executable(schema_perf_test SchemaPerfTest.cpp)
target_link_libraries(schema_perf_test ${PDAL_LIB_NAME})

endif()
