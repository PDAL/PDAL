
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

#set(PYTHON_LIBRARY ${PDAL_NO_AS_NEEDED_START}
#    ${PYTHON_LIBRARY}
#    ${PDAL_NO_AS_NEEDED_END})

include(${PDAL_CMAKE_DIR}/python.cmake)
include_directories(SYSTEM ${PYTHON_INCLUDE_DIR})


PDAL_ADD_LIBRARY(${PDAL_PLANG_LIB_NAME} ${plang_srcs} )
target_link_libraries(${PDAL_PLANG_LIB_NAME} ${PDAL_BASE_LIB_NAME} ${PYTHON_LIBRARY})
install(TARGETS ${PLANG_LIB_NAME}
    RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})


