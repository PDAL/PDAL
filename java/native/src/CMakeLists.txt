cmake_minimum_required(VERSION 2.8.0)

set(MAKE_COLOR_MAKEFILE ON)

# Define project and related variables
# (required by sbt-jni) please use semantic versioning
#

project (pdaljni)
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 4)
set(PROJECT_VERSION_PATCH 0)

set(PDAL_LIB_NAME pdalcpp)

if (APPLE)
  set(CMAKE_MACOSX_RPATH ON)
  SET(CMAKE_SKIP_BUILD_RPATH TRUE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif ()

# pdal requires to use cpp11, and a weird pdal flag to be sure that dynamic linking works
set(CMAKE_CXX_FLAGS "-std=c++11")

# Setup JNI
find_package(JNI REQUIRED)
if (JNI_FOUND)
    message (STATUS "JNI include directories: ${JNI_INCLUDE_DIRS}")
endif()

if (NOT PDAL_BUILD)
    find_package(PDAL 1.0.0 REQUIRED CONFIG)
endif()

# Include directories
include_directories(.)
include_directories(include)
include_directories(${JNI_INCLUDE_DIRS})
if (APPLE)
    include_directories(/usr/local/opt/libxml2/include/libxml2)
else()
    include_directories(/usr/include/libxml2)
endif()

# Sources
file(GLOB LIB_SRC
  "*.c"
  "*.cc"
  "*.cpp"
)

# Setup installation targets
# (required by sbt-jni) major version should always be appended to library name
#
set (LIB_NAME ${PROJECT_NAME}.${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
if (PDAL_BUILD)
    include_directories(${CMAKE_CURRENT_BINARY_DIR}/include/pdal)
    PDAL_ADD_LIBRARY(${LIB_NAME} ${LIB_SRC})
    target_link_libraries(${LIB_NAME} PUBLIC
        ${PDAL_BASE_LIB_NAME}
        ${PDAL_UTIL_LIB_NAME})
    target_include_directories(${LIB_NAME} PRIVATE
        ${PROJECT_BINARY_DIR}/include)
    install(TARGETS ${LIB_NAME} LIBRARY DESTINATION ${PDAL_LIB_INSTALL_DIR})
else ()
    add_library(${LIB_NAME} SHARED ${LIB_SRC})
    install(TARGETS ${LIB_NAME} LIBRARY DESTINATION .)
    target_link_libraries(${LIB_NAME} PRIVATE ${PDAL_LIB_NAME})
endif ()
