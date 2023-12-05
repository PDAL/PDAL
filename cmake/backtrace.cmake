set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
find_package(Libexecinfo QUIET)
find_package(Libunwind QUIET)
set(FPHSA_NAME_MISMATCHED 0)

if(LIBUNWIND_FOUND AND WITH_BACKTRACE)
    set(BACKTRACE_SOURCE ${PDAL_UTIL_DIR}/private/BacktraceUnwind.cpp)
    set(BACKTRACE_LIBRARIES ${LIBUNWIND_LIBRARIES} ${CMAKE_DL_LIBS})
elseif(LIBEXECINFO_FOUND AND WITH_BACKTRACE)
    set(BACKTRACE_SOURCE ${PDAL_UTIL_DIR}/private/BacktraceExecinfo.cpp)
    set(BACKTRACE_LIBRARIES ${LIBEXECINFO_LIBRARIES} ${CMAKE_DL_LIBS})
else()
    set(BACKTRACE_SOURCE ${PDAL_UTIL_DIR}/private/BacktraceNone.cpp)
endif()

