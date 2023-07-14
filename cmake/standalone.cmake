set(ROOT_DIR "${CMAKE_SOURCE_DIR}/../..")

get_filename_component(ROOT_DIR "${ROOT_DIR}" ABSOLUTE)

include(${ROOT_DIR}/cmake/common.cmake )
include(${ROOT_DIR}/cmake/libraries.cmake )
include(FeatureSummary)
find_package(PDAL COMPONENTS REQUIRED)
