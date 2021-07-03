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

if(LIBUNWIND_FOUND)
    set(BACKTRACE_SOURCE BacktraceUnwind.cpp)
    set(BACKTRACE_LIBRARIES ${LIBUNWIND_LIBRARIES})
elseif(LIBEXECINFO_FOUND)
    set(BACKTRACE_SOURCE BacktraceExecinfo.cpp)
    set(BACKTRACE_LIBRARIES ${LIBEXECINFO_LIBRARIES})
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

PDAL_ADD_FREE_LIBRARY(${PDAL_UTIL_LIB_NAME} SHARED ${PDAL_UTIL_SOURCES})
target_link_libraries(${PDAL_UTIL_LIB_NAME}
    PRIVATE
        ${BACKTRACE_LIBRARIES}
        ${PDAL_BOOST_LIB_NAME}
        ${CMAKE_DL_LIBS}
        ${CMAKE_THREAD_LIBS_INIT}
)
target_include_directories(${PDAL_UTIL_LIB_NAME} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost)

set_target_properties(${PDAL_UTIL_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)

set_property(GLOBAL PROPERTY _UTIL_INCLUDED TRUE)
