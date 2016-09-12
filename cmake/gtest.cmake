if (MSVC)
    # Since tests link CRT dynamically (/MD[d]), require gtest to
    #link dynamically too (default is /MT[d])
    option(gtest_force_shared_crt "Always use shared Visual C++ run-time DLL" ON)
endif()
add_subdirectory(vendor/gtest)

# gtest 1.7.0 has some CMake warnings, so we silence these by setting the
# following properties.
set_target_properties(gtest PROPERTIES
    MACOSX_RPATH ON
    LIBRARY_OUTPUT_DIRECTORY "${gtest_BINARY_DIR}/src"
)
set_target_properties(gtest_main PROPERTIES
    MACOSX_RPATH ON
    LIBRARY_OUTPUT_DIRECTORY "${gtest_BINARY_DIR}/src"
    )
