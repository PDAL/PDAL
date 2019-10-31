file(GLOB BASE_SRCS
    src/*.cpp
)

file(GLOB TEST_SRCS
    src/Test_C_Api.cpp)

list(REMOVE_ITEM BASE_SRCS ${TEST_SRCS})

PDAL_ADD_FREE_LIBRARY(lepcc STATIC ${BASE_SRCS})

set(LEPCC_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/src/include")

target_include_directories(lepcc PRIVATE
    src/include
)
