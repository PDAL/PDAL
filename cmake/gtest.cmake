if (MSVC)
    # Since tests link CRT dynamically (/MD[d]), require gtest to
    #link dynamically too (default is /MT[d])
    option(gtest_force_shared_crt "Always use shared Visual C++ run-time DLL" ON)
endif()

set(GOOGLETEST_VERSION 1.12.1)
add_subdirectory(vendor/gtest)

set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html


if (WITH_ABSEIL)
    find_package(absl REQUIRED)
    set(FPHSA_NAME_MISMATCHED 0)

    if (absl_FOUND)
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
