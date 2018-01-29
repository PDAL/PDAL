function(PDAL_TARGET_COMPILE_SETTINGS target)
    target_compile_definitions(${target} PUBLIC -DWIN32_LEAN_AND_MEAN)
    if (MSVC)
        target_compile_definitions(${target} PUBLIC -DBOOST_ALL_NO_LIB)

        # check for MSVC 8+
        if (NOT (MSVC_VERSION VERSION_LESS 1400))
            target_compile_definitions(${target} PUBLIC
                -D_CRT_SECURE_NO_DEPRECATE
                -D_CRT_SECURE_NO_WARNINGS
                -D_CRT_NONSTDC_NO_WARNING
                -D_SCL_SECURE_NO_WARNINGS
                -DNOMINMAX
                )
            target_compile_options(${target} PUBLIC
                # Nitro makes use of Exception Specifications, which results in
                # numerous warnings when compiling in MSVC. We will ignore them for
                # now.
                /wd4290
                /wd4800
                # Windows warns about integer narrowing like crazy and it's annoying.
                # In most cases the programmer knows what they're doing.  A good
                # static analysis tool would be better than turning this warning off.
                /wd4267
                )
        endif()

        # check for MSVC 9+
        if (NOT (MSVC_VERSION VERSION_LESS 1500))
            include(ProcessorCount)
            ProcessorCount(N)
            if(NOT N EQUAL 0)
                target_compile_options(${target} PRIVATE "/MP${N}")
            endif()
        endif()

        option(PDAL_USE_STATIC_RUNTIME "Use the static runtime" FALSE)
        if (PDAL_USE_STATIC_RUNTIME)
            target_compile_options(${target} PRIVATE /MT)
        endif()

    endif()
endfunction()

if (MSVC)
    if (CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W3"
            CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    endif()

    if (PDAL_USE_STATIC_RUNTIME)
            # Note that the CMake cache will still show /MD
            # http://www.cmake.org/Wiki/CMake_FAQ#Dynamic_Replace
            foreach(flag_var
                    CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
                    CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
                if(${flag_var} MATCHES "/MD")
                    string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
                endif(${flag_var} MATCHES "/MD")
            endforeach(flag_var)
    endif()
endif()

set(CMAKE_INCLUDE_PATH "c:/OSGeo4W64/include;$ENV{CMAKE_INCLUDE_PATH}")
set(CMAKE_LIBRARY_PATH "c:/OSGeo4W64/lib;$ENV{CMAKE_LIBRARY_PATH}")
set(CMAKE_PREFIX_PATH "c:/OSGeo4W64/cmake;$ENV{CMAKE_LIBRARY_PATH}")

#ABELL (& gadomski) - WHY?
set(PDAL_PLATFORM_WIN32 1)
set(WINSOCK_LIBRARY ws2_32)
