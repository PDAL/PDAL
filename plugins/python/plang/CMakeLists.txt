
set(plang_srcs
    BufferedInvocation.cpp
    Invocation.cpp
    Environment.cpp
    Plang.cpp
    Redirector.cpp
    Script.cpp
)

set(plang_headers
    BufferedInvocation.hpp
    Invocation.hpp
    Environment.hpp
    Plang.hpp
    Redirector.hpp
    Script.hpp
)

set(PYTHON_LIBRARY ${PDAL_NO_AS_NEEDED_START}
    ${PYTHON_LIBRARY}
    ${PDAL_NO_AS_NEEDED_END})
PDAL_ADD_LIBRARY(${PLANG_LIB_NAME} ${plang_srcs} ${plang_headers})
target_link_libraries(${PLANG_LIB_NAME} ${PDAL_BASE_LIB_NAME} ${PYTHON_LIBRARY})
install(TARGETS ${PLANG_LIB_NAME}
    RUNTIME DESTINATION ${PDAL_BIN_INSTALL_DIR}
    LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${PDAL_LIB_INSTALL_DIR})

if (WITH_TESTS)
    PDAL_ADD_TEST(plangtest FILES ../test/PLangTest.cpp
        LINK_WITH ${PLANG_LIB_NAME})

endif()
