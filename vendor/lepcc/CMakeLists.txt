file(GLOB BASE_SRCS
    src/*.cpp
)

file(GLOB TEST_SRCS
    src/Test_C_Api.cpp)

list(REMOVE_ITEM BASE_SRCS ${TEST_SRCS})

set(LEPCC_SRCS ${BASE_SRCS} PARENT_SCOPE)
