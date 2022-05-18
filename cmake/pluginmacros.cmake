macro(PDAL_CREATE_PLUGIN)
    set(options)
    set(oneValueArgs TYPE NAME VERSION)
    set(multiValueArgs SOURCES)
    cmake_parse_arguments(PDAL_CREATE_PLUGIN "${options}" "${oneValueArgs}" "${multiValueArgs}"
        ${ARGN} )

    string(TOLOWER ${PDAL_CREATE_PLUGIN_TYPE} TYPE)
    string(TOLOWER ${PDAL_CREATE_PLUGIN_NAME} NAME)
    set(VERSION ${PDAL_CREATE_PLUGIN_VERSION})
    set(SOURCES "${PDAL_CREATE_PLUGIN_SOURCES}")
    if (NOT TYPE STREQUAL "reader" AND
        NOT TYPE STREQUAL "writer" AND
        NOT TYPE STREQUAL "filter" AND
        NOT TYPE STREQUAL "kernel")
        message(FATAL_ERROR "Invalid plugin type '${TYPE}'. Must be 'reader', 'writer', 'filter' or 'kernel'.")
    endif()
    set(PROJECT_NAME ${NAME}_${TYPE})
    set(TARGET pdal_plugin_${TYPE}_${NAME})

    project(${PROJECT_NAME} VERSION ${VERSION} LANGUAGES CXX)

    add_library(${TARGET} SHARED ${SOURCES})
    set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD 17)
    set_property(TARGET ${TARGET} PROPERTY CXX_STANDARD_REQUIRED TRUE)
    target_include_directories(${TARGET} PRIVATE ${PDAL_INCLUDE_DIRS})
    target_link_libraries(${TARGET} PRIVATE ${PDAL_LIBRARIES})
endmacro()
