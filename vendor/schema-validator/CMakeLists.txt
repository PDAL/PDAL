get_property(EXISTS GLOBAL PROPERTY _PDALJSON_SCHEMA_INCLUDED)
if (EXISTS)
    return()
endif()

PDAL_ADD_FREE_LIBRARY(${JSON_SCHEMA_LIB_NAME} STATIC
    json-schema-draft7.json.cpp
    json-validator.cpp
    string-format-check.cpp
    json-uri.cpp
    json-patch.cpp
)

target_include_directories(${JSON_SCHEMA_LIB_NAME}
    PRIVATE
        ${NLOHMANN_INCLUDE_DIR}
)

# target_compile_definitions(${JSON_SCHEMA_LIB_NAME}
#     PRIVATE
#         JSON_SCHEMA_VALIDATOR
# )

# set_target_properties(${JSON_SCHEMA_LIB_NAME} PROPERTIES
#     VERSION "${PDAL_BUILD_VERSION}"
#     SOVERSION "${PDAL_API_VERSION}"
#     CLEAN_DIRECT_OUTPUT 1)

target_link_libraries(${JSON_SCHEMA_LIB_NAME})
set_property(GLOBAL PROPERTY _PDALJSON_SCHEMA_INCLUDED TRUE)