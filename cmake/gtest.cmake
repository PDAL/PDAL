if (MSVC)
    # Since tests link CRT dynamically (/MD[d]), require gtest to
    #link dynamically too (default is /MT[d])
    option(gtest_force_shared_crt "Always use shared Visual C++ run-time DLL" ON)
endif()

set(GOOGLETEST_VERSION 1.10.0)
add_subdirectory(vendor/gtest)

find_package(absl)
if (absl_FOUND)
    cmake_policy(SET CMP0079 NEW)
    target_compile_definitions(gtest PUBLIC GTEST_HAS_ABSL=1)
    target_compile_definitions(gtest_main PUBLIC GTEST_HAS_ABSL=1)
    target_link_libraries(gtest PRIVATE absl::algorithm
                                        absl::base
                                        absl::debugging
                                        absl::numeric
                                        absl::strings
                                        absl::utility
                                        absl::failure_signal_handler)
endif()
