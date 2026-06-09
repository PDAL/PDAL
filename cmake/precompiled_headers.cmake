set(PDAL_PRECOMPILED_HEADER
    "${ROOT_DIR}/pdal/pdal_pch.hpp")

function(pdal_target_precompile_headers target)
    if (NOT PDAL_USE_PRECOMPILED_HEADERS)
        return()
    endif()

    if (NOT COMMAND target_precompile_headers)
        message(FATAL_ERROR
            "PDAL_USE_PRECOMPILED_HEADERS requires a CMake version with "
            "target_precompile_headers().")
    endif()

    get_target_property(target_type ${target} TYPE)
    if (target_type STREQUAL "INTERFACE_LIBRARY")
        return()
    endif()

    target_precompile_headers(${target}
        PRIVATE
            "$<$<COMPILE_LANGUAGE:CXX>:${PDAL_PRECOMPILED_HEADER}>"
    )
endfunction()
