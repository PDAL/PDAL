set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

get_filename_component(ROOT_DIR "${ROOT_DIR}" ABSOLUTE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON CACHE BOOL
        "Build external PDAL plugins as shared libraries")
endif()

include(${ROOT_DIR}/cmake/common.cmake )
include(${ROOT_DIR}/cmake/libraries.cmake )
include(FeatureSummary)
find_package(PDAL COMPONENTS REQUIRED)
