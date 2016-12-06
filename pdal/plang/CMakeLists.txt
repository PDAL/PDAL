
set(plang_srcs
    Array.cpp
    BufferedInvocation.cpp
    Invocation.cpp
    Environment.cpp
    Redirector.cpp
    Script.cpp
)

include(${PDAL_CMAKE_DIR}/python.cmake)

PDAL_ADD_LIBRARY(${PDAL_PLANG_LIB_NAME} ${plang_srcs} )
set_target_properties(${PDAL_PLANG_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)
target_link_libraries(${PDAL_PLANG_LIB_NAME} PUBLIC
    ${PDAL_BASE_LIB_NAME}
    ${PDAL_UTIL_LIB_NAME}
    ${PYTHON_LIBRARY})
target_include_directories(${PDAL_PLANG_LIB_NAME} PRIVATE
    ${PYTHON_INCLUDE_DIR}
    ${PROJECT_BINARY_DIR}/include)
install(TARGETS ${PLANG_LIB_NAME}
    RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})


