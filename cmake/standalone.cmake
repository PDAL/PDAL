set(ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../..")

get_filename_component(ROOT_DIR "${ROOT_DIR}" ABSOLUTE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${ROOT_DIR}/cmake/common.cmake )
include(${ROOT_DIR}/cmake/libraries.cmake )
include(FeatureSummary)
find_package(PDAL COMPONENTS REQUIRED)
