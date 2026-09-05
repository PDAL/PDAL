
# Find the sphinx and jupyter-book commands
find_program(JUPYTERBOOK jupyter-book)
if (NOT JUPYTERBOOK)
    message(FATAL_ERROR 
        "Building documentation requires the 'jupyter-book' "
        "command to be discoverable by cmake."
    )
endif()

# Find sed command for jupter-book config - only really necessary for readthedocs
if(UNIX AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_program(SED_EXECUTABLE gsed)
else()
    find_program(SED_EXECUTABLE sed)
endif()

find_package(Doxygen)
if (NOT DOXYGEN_FOUND)
    message("Could not find Doxygen. API documentation will not be built.")
endif()
find_package(Python COMPONENTS Interpreter REQUIRED)

# Optional: only used by the "logo_assets" target (see doc/CMakeLists.txt)
# to regenerate the checked-in PNG logo assets from the master SVGs under
# doc/_static/logo after a maintainer edits one. Not required to build
# the docs -- the PNGs the docs actually reference are committed to the
# repo like any other doc asset.
find_program(RSVG_CONVERT_EXECUTABLE rsvg-convert)