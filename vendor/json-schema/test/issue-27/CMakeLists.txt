add_test_simple_schema(Issue::27
                       ${CMAKE_CURRENT_SOURCE_DIR}/schema.json
                       ${CMAKE_CURRENT_SOURCE_DIR}/instance.json)
set_tests_properties(Issue::27
                     PROPERTIES
                         WILL_FAIL 1)


