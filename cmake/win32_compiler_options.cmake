if (MSVC)
    set(PDAL_COMPILER_MSVC 1)
    if (MSVC12)
      set(PDAL_COMPILER_VC12 1)
    elseif (MSVC11)
      set(PDAL_COMPILER_VC11 1)
    elseif (MSVC10)
      set(PDAL_COMPILER_VC10 1)
    elseif (MSVC9)
      set(PDAL_COMPILER_VC9 1)
    elseif (MSVC8)
      set(PDAL_COMPILER_VC8 1)
    endif()

    add_definitions(-DBOOST_ALL_NO_LIB)

    # check for MSVC 8+
    if (NOT (MSVC_VERSION VERSION_LESS 1400))
        add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        add_definitions(-D_CRT_NONSTDC_NO_WARNING)
        add_definitions(-D_SCL_SECURE_NO_WARNINGS)
        add_definitions(-DNOMINMAX)

        # Nitro makes use of Exception Specifications, which results in
        # numerous warnings when compiling in MSVC. We will ignore them for
        # now.
        add_definitions("/wd4290")
	add_definitions("/wd4800")

        # Windows still warns about nameless struct/union, but we assume
        # that all of our compilers support this
        #add_definitions("/wd4201")
    endif()

    if (CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W3"
               CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3")
    endif()

    # check for MSVC 9+
    if (NOT (MSVC_VERSION VERSION_LESS 1500))
        include(ProcessorCount)
        ProcessorCount(N)
        if(NOT N EQUAL 0)
            set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   /MP${N}")
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${N}")
        endif()
    endif()

    option(PDAL_USE_STATIC_RUNTIME "Use the static runtime" FALSE)
    if (PDAL_USE_STATIC_RUNTIME)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MT")

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

endif(MSVC)
add_definitions(-DWIN32_LEAN_AND_MEAN)

# note we default to debug mode
#if(NOT MSVC_IDE)
#  if(NOT CMAKE_BUILD_TYPE)
#  set(CMAKE_BUILD_TYPE Debug CACHE STRING
#    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel" FORCE)
#  endif()
#  message(STATUS "Setting PDAL build type - ${CMAKE_BUILD_TYPE}")
#endif()

set(CMAKE_INCLUDE_PATH "c:/OSGeo4W64/include;$ENV{CMAKE_INCLUDE_PATH}")
set(CMAKE_LIBRARY_PATH "c:/OSGeo4W64/lib;$ENV{CMAKE_LIBRARY_PATH}")
set(CMAKE_PREFIX_PATH "c:/OSGeo4W64/cmake;$ENV{CMAKE_LIBRARY_PATH}")

#ABELL - WHY?
set(PDAL_PLATFORM_WIN32 1)
