# This is a simple project that tests using cmake to load the installed libraries
cmake_minimum_required(VERSION 3.2)

project(cmake_install_test LANGUAGES CXX)

set(PROJECT_VERSION 1.0.0)

# Find the nlohmann_json and the validator package
set(CMAKE_FIND_DEBUG_MODE ON)
find_package(nlohmann_json REQUIRED)
find_package(nlohmann_json_schema_validator REQUIRED)

# Add simple json-schema-validator-executable
add_executable(json-schema-validate ${CMAKE_CURRENT_SOURCE_DIR}/../../../app/json-schema-validate.cpp)
target_link_libraries(json-schema-validate nlohmann_json_schema_validator)

enable_testing()

# Add built-in tests function needed for issues
set(PIPE_IN_TEST_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/../../test-pipe-in.sh)
function(add_test_simple_schema name schema instance)
    add_test(
        NAME ${name}
        COMMAND ${PIPE_IN_TEST_SCRIPT}
            $<TARGET_FILE:json-schema-validate>
            ${schema}
            ${instance}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

# Run tests for issues 9, 12, 27, 48, 54
foreach(NUMBER "9" "12" "27" "48" "54")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../../issue-${NUMBER}" "${CMAKE_CURRENT_BINARY_DIR}/issue-${NUMBER}" EXCLUDE_FROM_ALL)
endforeach()
