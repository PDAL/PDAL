
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