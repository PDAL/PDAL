#
# N Lohmann JSON handler
#

if(NOT CMAKE_REQUIRED_QUIET)
    # CMake 3.17+ use CHECK_START/CHECK_PASS/CHECK_FAIL
    message(STATUS "Looking for nlohmann")
endif()


find_package(nlohmann_json QUIET)
set(USE_EXTERNAL_NLOHMANN_DEFAULT OFF)
if(nlohmann_json_FOUND)
    if(NOT CMAKE_REQUIRED_QUIET)
        message(STATUS "Looking for local nlohmann_json - found (${Nlohmann_VERSION})")
    endif()
    return()
else()
    if(NOT CMAKE_REQUIRED_QUIET)
        message(STATUS "Looking for local nlohmann_json - not found")
    endif()
    set(USE_EXTERNAL_NLOHMANN_DEFAULT ON)
endif()

message(STATUS "Should we use an external nlohmann_json by default? ${USE_EXTERNAL_NLOHMANN_DEFAULT} ")
option(USE_EXTERNAL_NLOHMANN "Use an external nlohmann JSON library" ${USE_EXTERNAL_NLOHMANN_DEFAULT})

set(NLOHMANN_REQUESTED_VERSION "3.11.3")
if(USE_EXTERNAL_NLOHMANN)
    message(STATUS "Using external nlohmann_json")
    find_package(nlohmann_json "${NLOHMANN_REQUESTED_VERSION}" CONFIG)

    if(nlohmann_json_FOUND)
        message(STATUS "External nlohmann_json found in environment")
        return()
    else()
        if(NOT CMAKE_REQUIRED_QUIET)
            message(STATUS "External nlohmann_json not found locally. Fetching remote nlohmann_json from GitHub")
        endif()
        if(POLICY CMP0135)
            cmake_policy(SET CMP0135 NEW)  # for DOWNLOAD_EXTRACT_TIMESTAMP option
        endif()
        include(FetchContent)
        FetchContent_Declare(nlohmann
            URL "https://github.com/nlohmann/json/releases/download/v${NLOHMANN_REQUESTED_VERSION}/json.tar.xz")

        if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28.0")
            FetchContent_MakeAvailable(nlohmann)
        else()
            # Pre CMake 3.28 workaround to prevent installing files
            FetchContent_GetProperties(nlohmann)
            if(NOT nlohmann_POPULATED)
                FetchContent_Populate(nlohmann)
                add_subdirectory(${nlohmann_SOURCE_DIR} ${nlohmann_BINARY_DIR} EXCLUDE_FROM_ALL)
            endif()
        endif()
    endif()
endif()



