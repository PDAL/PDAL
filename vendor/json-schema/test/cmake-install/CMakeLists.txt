# Configure install script
configure_file(test.sh.in
               ${CMAKE_CURRENT_BINARY_DIR}/test.sh @ONLY)

get_filename_component(TEST_NAME
   ${CMAKE_CURRENT_SOURCE_DIR}
   NAME)


# this build test only works, if nlohmann-json was found via a cmake-package
if(TARGET nlohmann_json::nlohmann_json)
    add_test(NAME Build::${TEST_NAME}
             COMMAND ${CMAKE_CURRENT_BINARY_DIR}/test.sh
             WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()
