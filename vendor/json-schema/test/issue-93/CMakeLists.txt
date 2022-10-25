
add_executable(issue-93 issue-93.cpp)
target_link_libraries(issue-93 nlohmann_json_schema_validator)

add_test(NAME issue-93
         COMMAND issue-93
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
