
set(plang_srcs
    Array.cpp
    BufferedInvocation.cpp
    Invocation.cpp
    Environment.cpp
    Redirector.cpp
    Script.cpp
)

set(plang_headers
    "${PDAL_HEADERS_DIR}/plang/Array.hpp"
    "${PDAL_HEADERS_DIR}/plang/BufferedInvocation.hpp"
    "${PDAL_HEADERS_DIR}/plang/Invocation.hpp"
    "${PDAL_HEADERS_DIR}/plang/Environment.hpp"
    "${PDAL_HEADERS_DIR}/plang/Redirector.hpp"
    "${PDAL_HEADERS_DIR}/plang/Script.hpp"
)

include(${PDAL_CMAKE_DIR}/python.cmake)
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})

PDAL_ADD_LIBRARY(${PDAL_PLANG_LIB_NAME} ${plang_srcs} )
set_target_properties(${PDAL_PLANG_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(${PDAL_PLANG_LIB_NAME} ${PDAL_BASE_LIB_NAME} ${PYTHON_LIBRARY})
install(TARGETS ${PLANG_LIB_NAME}
    RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})


