# CMake configuration for PROJ unit tests
# External GTest provided by (e.g.) libgtest-dev

set(MIN_GTest_VERSION "1.8.1")

if(NOT CMAKE_REQUIRED_QUIET)
  # CMake 3.17+ use CHECK_START/CHECK_PASS/CHECK_FAIL
  message(STATUS "Looking for GTest")
endif()
find_package(GTest QUIET)
set(USE_EXTERNAL_GTEST_DEFAULT OFF)
if(GTest_FOUND)
  if(NOT CMAKE_REQUIRED_QUIET)
    message(STATUS "Looking for GTest - found (${GTest_VERSION})")
  endif()
  if(GTest_VERSION VERSION_LESS MIN_GTest_VERSION)
    message(WARNING "External GTest version is too old")
  else()
    set(USE_EXTERNAL_GTEST_DEFAULT ON)
  endif()
else()
  if(NOT CMAKE_REQUIRED_QUIET)
    message(STATUS "Looking for GTest - not found")
  endif()
endif()

option(USE_EXTERNAL_GTEST
  "Compile against external GTest"
  ${USE_EXTERNAL_GTEST_DEFAULT}
)

if(USE_EXTERNAL_GTEST)

  if(NOT GTest_FOUND)
    message(SEND_ERROR "External GTest >= ${MIN_GTest_VERSION} not found, \
      skipping some tests")
    # exit the remainder of this file
    return()
  endif()
  message(STATUS "Using external GTest")

  # CMake < 3.20.0 uses GTest::GTest
  # CMake >= 3.20 uses GTest::gtest, and deprecates GTest::GTest
  # so for older CMake, create an alias from GTest::GTest to GTest::gtest
  if(NOT TARGET GTest::gtest)
    add_library(GTest::gtest INTERFACE IMPORTED)
    set_target_properties(GTest::gtest PROPERTIES
                          INTERFACE_LINK_LIBRARIES "GTest::GTest")
  endif()

else()

  message(STATUS "Fetching GTest from GitHub ...")

  # Add Google Test
  #
  # See https://github.com/google/googletest/blob/main/googletest/README.md

  if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)  # for DOWNLOAD_EXTRACT_TIMESTAMP option
  endif()

  set(GTEST_VERSION "1.15.2")

  include(FetchContent)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v${GTEST_VERSION}.zip
    EXCLUDE_FROM_ALL  # ignored before CMake 3.28
  )

  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.28.0")
    FetchContent_MakeAvailable(googletest)
  else()
    # Pre CMake 3.28 workaround to prevent installing files
    FetchContent_GetProperties(googletest)
    if(NOT googletest_POPULATED)
      FetchContent_Populate(googletest)
      add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
    endif()
  endif()
endif()

