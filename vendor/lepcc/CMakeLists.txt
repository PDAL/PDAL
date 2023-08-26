get_property(EXISTS GLOBAL PROPERTY _PDAL_LEPCC_INCLUDED)
if (EXISTS)
    return()
endif()

file(GLOB BASE_SRCS
    src/*.cpp
)

file(GLOB TEST_SRCS
    src/Test_C_Api.cpp)

list(REMOVE_ITEM BASE_SRCS ${TEST_SRCS})

PDAL_ADD_FREE_LIBRARY(${PDAL_LEPCC_LIB_NAME} STATIC ${BASE_SRCS})

set(LEPCC_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/src/include")

target_include_directories(${PDAL_LEPCC_LIB_NAME} PRIVATE
    src/include
)


set_property(GLOBAL PROPERTY _PDAL_LEPCC_INCLUDED TRUE)
