#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _UTIL_INCLUDED)
if(EXISTS)
    return()
endif()

include(${PDAL_CMAKE_DIR}/execinfo.cmake)
include(${PDAL_CMAKE_DIR}/threads.cmake)
include(${PDAL_CMAKE_DIR}/unwind.cmake)
include(${PDAL_CMAKE_DIR}/utfcpp.cmake)

if(LIBUNWIND_FOUND)
    set(BACKTRACE_SOURCE BacktraceUnwind.cpp)
    set(BACKTRACE_LIBRARIES ${LIBUNWIND_LIBRARIES} ${CMAKE_DL_LIBS})
elseif(LIBEXECINFO_FOUND)
    set(BACKTRACE_SOURCE BacktraceExecinfo.cpp)
    set(BACKTRACE_LIBRARIES ${LIBEXECINFO_LIBRARIES} ${CMAKE_DL_LIBS})
else()
    set(BACKTRACE_SOURCE BacktraceNone.cpp)
endif()

set(PDAL_UTIL_SOURCES
    "${PDAL_UTIL_DIR}/Bounds.cpp"
    "${PDAL_UTIL_DIR}/Charbuf.cpp"
    "${PDAL_UTIL_DIR}/FileUtils.cpp"
    "${PDAL_UTIL_DIR}/Georeference.cpp"
    "${PDAL_UTIL_DIR}/ThreadPool.cpp"
    "${PDAL_UTIL_DIR}/Utils.cpp"
    "${PDAL_UTIL_DIR}/Backtrace.cpp"
    "${PDAL_UTIL_DIR}/Random.cpp"
    "${PDAL_UTIL_DIR}/private/${BACKTRACE_SOURCE}"
)

add_library(${PDAL_UTIL_LIB_NAME} OBJECT ${PDAL_UTIL_SOURCES})
set_property(TARGET ${PDAL_UTIL_LIB_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PDAL_UTIL_LIB_NAME}
    PRIVATE
        ${BACKTRACE_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${UTFCPP_LIB_NAME}
)

target_link_libraries(${PDAL_UTIL_LIB_NAME}
    PRIVATE
        ${WINSOCK_LIBRARY}
)

target_include_directories(${PDAL_UTIL_LIB_NAME}
    PRIVATE
        ${PDAL_INCLUDE_DIR}
        ${UTFCPP_INCLUDE_DIR}
        ${NLOHMANN_INCLUDE_DIR}
)

#set_target_properties(${PDAL_UTIL_LIB_NAME} PROPERTIES
#    VERSION "${PDAL_BUILD_VERSION}"
#    SOVERSION "${PDAL_API_VERSION}"
#    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)

# Compatibility for GCC8: std::filesystem requires linking with libstdc++fs
target_link_libraries(
  ${PDAL_UTIL_LIB_NAME}
  PRIVATE
  "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:-lstdc++fs>"
)
