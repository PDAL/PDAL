# EXAMPLE support
#

function(pdal_example_compile_settings target)
    if (WITH_EXAMPLES)
        target_include_directories(${target} PRIVATE ${ROOT_DIR}/pdal ${ROOT_DIR})
    endif()
endfunction()
