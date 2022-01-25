include(FetchContent)
FetchContent_Declare(
  googletest
  # Specify the commit you depend on and update it regularly.
  URL https://github.com/google/googletest/archive/f45d5865ed0b2b8912244627cdf508a24cc6ccb4.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

find_package(absl QUIET)
set(FPHSA_NAME_MISMATCHED 0)

if (absl_FOUND)
    if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.13.0")
        cmake_policy(SET CMP0079 NEW)
    endif()
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
