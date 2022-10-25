add_test_simple_schema(Issue::143-1
                       ${CMAKE_CURRENT_SOURCE_DIR}/schema.json
                       ${CMAKE_CURRENT_SOURCE_DIR}/instance-fail-1.json)
add_test_simple_schema(Issue::143-a
                       ${CMAKE_CURRENT_SOURCE_DIR}/schema.json
                       ${CMAKE_CURRENT_SOURCE_DIR}/instance-fail-a.json)
add_test_simple_schema(Issue::143-ok
                       ${CMAKE_CURRENT_SOURCE_DIR}/schema.json
                       ${CMAKE_CURRENT_SOURCE_DIR}/instance.json)

set_tests_properties(Issue::143-1 Issue::143-a
                     PROPERTIES
                     WILL_FAIL 1)
