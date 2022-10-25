set(PIPE_IN_TEST_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/test-pipe-in.sh)

# built-in tests
function(add_test_simple_schema name schema instance)
    add_test(
        NAME ${name}
        COMMAND ${PIPE_IN_TEST_SCRIPT}
            $<TARGET_FILE:json-schema-validate>
            ${schema}
            ${instance}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

file(GLOB TEST_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/*)

foreach(DIR ${TEST_DIRS})
    if(IS_DIRECTORY ${DIR})
        add_subdirectory(${DIR})
    endif()
endforeach()

add_executable(uri uri.cpp)
target_link_libraries(uri nlohmann_json_schema_validator)
add_test(NAME uri COMMAND uri)

add_executable(errors errors.cpp)
target_link_libraries(errors nlohmann_json_schema_validator)
add_test(NAME errors COMMAND errors)

add_executable(issue-70 issue-70.cpp)
target_link_libraries(issue-70 nlohmann_json_schema_validator)
add_test(NAME issue-70 COMMAND issue-70)

add_executable(issue-70-root-schema-constructor issue-70-root-schema-constructor.cpp)
target_link_libraries(issue-70-root-schema-constructor nlohmann_json_schema_validator)
add_test(NAME issue-70-root-schema-constructor COMMAND issue-70-root-schema-constructor)

add_executable(issue-25-default-values issue-25-default-values.cpp)
target_link_libraries(issue-25-default-values nlohmann_json_schema_validator)
add_test(NAME issue-25-default-values COMMAND issue-25-default-values)

add_executable(issue-98 issue-98.cpp)
target_link_libraries(issue-98 nlohmann_json_schema_validator)
add_test(NAME issue-98-erase-exception-unknown-keywords COMMAND issue-98)

# Unit test for string format checks
add_executable(string-format-check-test string-format-check-test.cpp)
target_include_directories(string-format-check-test PRIVATE ${PROJECT_SOURCE_DIR}/src/)
target_link_libraries(string-format-check-test nlohmann_json_schema_validator)

add_test(NAME string-format-check-test COMMAND string-format-check-test)

# Unit test for json-patch
add_executable(json-patch json-patch.cpp)
target_include_directories(json-patch PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(json-patch nlohmann_json_schema_validator)
add_test(NAME json-patch COMMAND json-patch)

# Unit test for format checker fail at schema parsing time
add_executable(issue-117-format-error issue-117-format-error.cpp)
target_link_libraries(issue-117-format-error nlohmann_json_schema_validator)
add_test(NAME issue-117-format-error COMMAND issue-117-format-error)

add_executable(binary-validation binary-validation.cpp)
target_include_directories(binary-validation PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(binary-validation PRIVATE nlohmann_json_schema_validator)
add_test(NAME binary-validation COMMAND binary-validation)

add_executable(issue-149-entry-selection issue-149-entry-selection.cpp)
target_link_libraries(issue-149-entry-selection PRIVATE nlohmann_json_schema_validator)
add_test(NAME issue-149-entry-selection COMMAND issue-149-entry-selection)

add_executable(issue-189-default-values issue-189-default-values.cpp)
target_link_libraries(issue-189-default-values nlohmann_json_schema_validator)
add_test(NAME issue-189-default-values COMMAND issue-189-default-values)
