if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 4.7)
        set(PDAL_CXX_STANDARD "-std=c++0x")
    elseif(${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 7.0)
        set(PDAL_CXX_STANDARD "-std=c++11")
    else()
        set(PDAL_CXX_STANDARD
            -std=c++11
            -Wno-implicit-fallthrough
            -Wno-int-in-bool-context
            -Wno-dangling-else
            -Wno-noexcept-type)
    endif()
    set(PDAL_COMPILER_GCC 1)
elseif (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(PDAL_CXX_STANDARD "-std=c++11")
    set(PDAL_COMPILER_CLANG 1)
else()
    message(FATAL_ERROR "Unsupported C++ compiler")
endif()

function(PDAL_TARGET_COMPILE_SETTINGS target)
    target_compile_options(${target} PUBLIC
        ${PDAL_CXX_STANDARD}
        -Wextra
        -Wpedantic
        -Werror
        -Wall
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wpointer-arith
        -Wcast-align
        -Wcast-qual
        -Wredundant-decls
        -Wno-long-long
        -Wno-unknown-pragmas
        -Wno-deprecated-declarations
        )
    target_include_directories(${target} SYSTEM PUBLIC /usr/local/include)
endfunction()
