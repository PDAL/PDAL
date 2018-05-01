#
# Make sure we don't attempt to add a library more than once
#
get_property(EXISTS GLOBAL PROPERTY _PDALBOOST_INCLUDED)
if (EXISTS)
    return()
endif()

file(GLOB PDAL_BOOST_SOURCES
    "libs/system/src/*"
    "libs/filesystem/src/*"
)

# If we're not on WIN32, remove windows sources to eliminate warning message.
if (NOT WIN32)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    file(GLOB REMOVE_SOURCES
        "libs/program_options/src/winmain.cpp"
        "libs/filesystem/src/windows_file_codecvt.*"
    )
    foreach(REMOVE_SOURCE IN LISTS REMOVE_SOURCES)
        list (REMOVE_ITEM PDAL_BOOST_SOURCES ${REMOVE_SOURCE})
    endforeach()
endif()

add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)

PDAL_ADD_FREE_LIBRARY(${PDAL_BOOST_LIB_NAME} STATIC "${PDAL_BOOST_SOURCES}")
set_target_properties(${PDAL_BOOST_LIB_NAME} PROPERTIES
    VERSION "${PDAL_BUILD_VERSION}"
    SOVERSION "${PDAL_API_VERSION}"
    CLEAN_DIRECT_OUTPUT 1)
target_include_directories(${PDAL_BOOST_LIB_NAME} PRIVATE
    ${PDAL_VENDOR_DIR}/pdalboost)

set_property(GLOBAL PROPERTY _PDALBOOST_INCLUDED TRUE)

