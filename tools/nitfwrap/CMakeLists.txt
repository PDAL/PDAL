include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)
include(${PDAL_CMAKE_DIR}/nitro.cmake)

set(PDAL_NITF_DIR ${ROOT_DIR}/plugins/nitf/io)

add_executable(nitfwrap
    NitfWrap.cpp
    ${PDAL_NITF_DIR}/MetadataReader.cpp
    ${PDAL_NITF_DIR}/NitfFileReader.cpp
    ${PDAL_NITF_DIR}/NitfFileWriter.cpp
    ${PDAL_NITF_DIR}/tre_plugins.cpp
)
pdal_target_compile_settings(nitfwrap)
add_dependencies(nitfwrap generate_dimension_hpp)
target_link_libraries(nitfwrap PRIVATE
    ${PDAL_BASE_LIB_NAME}
    ${PDAL_UTIL_LIB_NAME}
    ${NITRO_LIBRARIES}
)

target_include_directories(nitfwrap PRIVATE
    ${PROJECT_BINARY_DIR}/include
    ${PDAL_INCLUDE_DIR}
    ${ROOT_DIR}
)

if (WITH_TESTS)
    PDAL_ADD_TEST(nitfwrap_test
        FILES NitfWrapTest.cpp
    )
endif()

