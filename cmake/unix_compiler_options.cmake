set(PDAL_COMMON_CXX_FLAGS "-Wextra -Wall -Wno-unused-parameter -Wno-unused-variable -Wpointer-arith -Wcast-align -Wcast-qual -Wredundant-decls -Wno-long-long -Wno-unknown-pragmas -Wno-deprecated-declarations -isystem /usr/local/include"
)

if (${CMAKE_CXX_COMPILER_ID} MATCHES "GNU")
    if (${CMAKE_CXX_COMPILER_VERSION} VERSION_LESS 4.7)
        set(PDAL_CXX_STANDARD "-std=c++0x")
    else()
        set(PDAL_CXX_STANDARD "-std=c++11")
    endif()
    set(PDAL_COMPILER_GCC 1)
elseif (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang")
    set(PDAL_CXX_STANDARD "-std=c++11")
    set(PDAL_COMPILER_CLANG 1)
else()
    message(FATAL_ERROR "Unsupported C++ compiler")
endif()

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${PDAL_COMMON_CXX_FLAGS} ${PDAL_CXX_STANDARD}")
