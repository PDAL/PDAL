function(pdal_target_compile_settings target)
    set_property(TARGET ${target} PROPERTY CXX_STANDARD 17)
    set_property(TARGET ${target} PROPERTY CXX_STANDARD_REQUIRED TRUE)
    if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
        #
        # VERSION_GREATER_EQUAL doesn't come until cmake 3.7
        #
        if (NOT ${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 7.0)
            target_compile_options(${target}
                PRIVATE $<$<COMPILE_LANGUAGE:C>:-Wno-except-type>
                        $<$<COMPILE_LANGUAGE:C>:-Wno-class-memaccess>
            )
            target_compile_options(${target} PRIVATE
                -Wno-implicit-fallthrough
                -Wno-int-in-bool-context
                -Wno-dangling-else
            )
        endif()
        set(PDAL_COMPILER_GCC 1)
    elseif (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
        set(PDAL_COMPILER_CLANG 1)
    else()
        message(FATAL_ERROR "Unsupported C++ compiler")
    endif()

    target_compile_options(${target} PRIVATE
        ${PDAL_CXX_STANDARD}
        -Wall
        -Wextra
        -Wpointer-arith
        -Wcast-align
        -Wcast-qual
        -Wno-error=parentheses
        -Wno-error=cast-qual
        -Wredundant-decls

        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-long-long
        -Wno-unknown-pragmas
        -Wno-deprecated-declarations
    )
    if (PDAL_COMPILER_CLANG)
        target_compile_options(${target} PRIVATE
            -Wno-unknown-warning-option
        )
    endif()
endfunction()



function(pdal_lib_compile_settings target)
    pdal_target_compile_settings(${target})

    target_compile_options(${target} PRIVATE
        -fvisibility=hidden
        -fvisibility-inlines-hidden)
endfunction()
