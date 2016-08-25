include(${ROOT_DIR}/cmake/common.cmake NO_POLICY_SCOPE)
include(${PDAL_CMAKE_DIR}/nitro.cmake)

set(PDAL_NITF_DIR ${ROOT_DIR}/plugins/nitf/io)

set (SOURCES
    NitfWrap.cpp
    ${PDAL_NITF_DIR}/MetadataReader.cpp
    ${PDAL_NITF_DIR}/NitfFileReader.cpp
    ${PDAL_NITF_DIR}/NitfFileWriter.cpp
    ${PDAL_NITF_DIR}/tre_plugins.cpp
)

set (HEADERS
    NitfWrap.hpp
    ${PDAL_NITF_DIR}/MetadataReader.hpp
    ${PDAL_NITF_DIR}/NitfFileReader.hpp
    ${PDAL_NITF_DIR}/NitfFileWriter.hpp
    ${PDAL_NITF_DIR}/tre_plugins.hpp
)

add_executable(nitfwrap ${SOURCES} ${HEADERS})
add_dependencies(nitfwrap generate_dimension_hpp)
target_link_libraries(nitfwrap
    ${PDAL_BASE_LIB_NAME}
    ${PDAL_UTIL_LIB_NAME}
    ${NITRO_LIBRARIES}
)
target_include_directories(nitfwrap
    PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/include"
    "${PDAL_INCLUDE_DIR}"
    "${PDAL_NITF_DIR}"
)


if (WITH_TESTS)
    PDAL_ADD_TEST(
        nitfwrap_test
        FILES NitfWrapTest.cpp
    )
endif()

