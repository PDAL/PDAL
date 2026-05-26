include_guard(GLOBAL)

set(MIN_GTest_VERSION "1.12.1")

if(USE_EXTERNAL_GTEST)
    if(NOT CMAKE_REQUIRED_QUIET)
        message(STATUS "Looking for GTest")
    endif()
    find_package(GTest QUIET CONFIG)

    if(GTest_FOUND)
        if(NOT CMAKE_REQUIRED_QUIET)
            if(GTest_VERSION)
                message(STATUS "Looking for GTest - found (${GTest_VERSION})")
            else()
                message(STATUS "Looking for GTest - found")
            endif()
        endif()
    else()
        if(NOT CMAKE_REQUIRED_QUIET)
            message(STATUS "Looking for GTest - not found")
        endif()
    endif()

    if(NOT GTest_FOUND)
        message(SEND_ERROR "External GTest requested but not found")
        return()
    endif()

    if(GTest_VERSION AND GTest_VERSION VERSION_LESS MIN_GTest_VERSION)
        message(SEND_ERROR "External GTest version must be at least ${MIN_GTest_VERSION}")
        return()
    endif()

    message(STATUS "Using external GTest from build environment")

    if(NOT TARGET GTest::gtest AND TARGET GTest::GTest)
        add_library(GTest::gtest INTERFACE IMPORTED)
        set_target_properties(GTest::gtest PROPERTIES
            INTERFACE_LINK_LIBRARIES "GTest::GTest")
    endif()

    if(NOT TARGET GTest::gtest_main AND TARGET GTest::Main)
        add_library(GTest::gtest_main INTERFACE IMPORTED)
        set_target_properties(GTest::gtest_main PROPERTIES
            INTERFACE_LINK_LIBRARIES "GTest::Main")
    endif()

    if(NOT TARGET GTest::gtest OR NOT TARGET GTest::gtest_main)
        message(SEND_ERROR
            "External GTest installation must provide GTest::gtest and GTest::gtest_main targets")
        return()
    endif()
else()
    message(STATUS "Using vendored GTest from source tree")

    if(MSVC)
        # Since tests link CRT dynamically (/MD[d]), require gtest to
        # link dynamically too (default is /MT[d]).
        option(gtest_force_shared_crt "Always use shared Visual C++ run-time DLL" ON)
    endif()

    include_directories(${ROOT_DIR}/vendor/gtest/include
        ${ROOT_DIR}/vendor/gtest)

    set(GOOGLETEST_VERSION 1.12.1)
    add_subdirectory(vendor/gtest)

    if(NOT TARGET GTest::gtest)
        add_library(GTest::gtest ALIAS gtest)
    endif()

    if(NOT TARGET GTest::gtest_main)
        add_library(GTest::gtest_main ALIAS gtest_main)
    endif()

    set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html

    if(WITH_ABSEIL)
        find_package(absl REQUIRED)
        set(FPHSA_NAME_MISMATCHED 0)

        if(absl_FOUND)
            find_package(re2 REQUIRED)

            if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13.0")
                cmake_policy(SET CMP0079 NEW)
            endif()
            target_compile_definitions(gtest PUBLIC GTEST_HAS_ABSL=1)
            target_compile_definitions(gtest_main PUBLIC GTEST_HAS_ABSL=1)
            target_link_libraries(gtest PRIVATE absl::algorithm
                                                absl::base
                                                absl::flags
                                                absl::flags_internal
                                                absl::flags_usage
                                                absl::flags_commandlineflag
                                                absl::flags_parse
                                                absl::debugging
                                                absl::numeric
                                                absl::strings
                                                absl::utility
                                                absl::failure_signal_handler)
            target_link_libraries(gtest PUBLIC re2::re2)
        endif()
    endif()
endif()
