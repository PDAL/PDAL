#
# NITRO support
#

find_package(Nitro QUIET 2.6 REQUIRED)
if (NITRO_FOUND)
    include_directories(${NITRO_INCLUDE_DIR})
    include_directories(${NITRO_INCLUDE_DIR}/nitro/c++)
    include_directories(${NITRO_INCLUDE_DIR}/nitro/c)
    add_definitions("-D_REENTRANT")
    if (WIN32)
        add_definitions("-DSIZEOF_SIZE_T=4")
        add_definitions("-DIMPORT_NITRO_API")
    else()
        add_definitions("-D__POSIX")
    endif()
endif()
