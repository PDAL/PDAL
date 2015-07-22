# Default to using static, multithreaded libraries for
# linking under MSVC.  This is because we show users how to
# use boostpro.com installer and install those options when linking
# on windows in the compilation documentation.
if(WIN32)
    if (MSVC)
        set(Boost_USE_STATIC_LIBS   OFF)
        set(Boost_USE_MULTITHREADED ON)

        if (PDAL_USE_STATIC_RUNTIME)
            set(Boost_USE_STATIC_RUNTIME ON)
        endif(PDAL_USE_STATIC_RUNTIME)
    endif(MSVC)
endif(WIN32)

find_package(Boost 1.53 REQUIRED COMPONENTS program_options iostreams filesystem system thread)
set_package_properties(Boost PROPERTIES TYPE REQUIRED
    PURPOSE "Boost provides fundamental functionality for PDAL")
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})

  # make these available for the user to set.
  mark_as_advanced(CLEAR Boost_INCLUDE_DIR)
  mark_as_advanced(CLEAR Boost_LIBRARY_DIRS)
  link_directories(${Boost_LIBRARY_DIRS})
endif(Boost_FOUND)
