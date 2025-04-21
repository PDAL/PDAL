
# Find the sphinx and jupyter-book commands
find_program(JUPYTERBOOK jupyter-book)
find_program(SPHINX_BUILD sphinx-build)
if (NOT JUPYTERBOOK)
    message(FATAL_ERROR 
        "Building documentation requires the 'jupyter-book' "
        "command to be discoverable by cmake."
    )
endif()
if (NOT SPHINX_BUILD)
    message(FATAL_ERROR 
        "Building documentation requires the 'sphinx-build' "
        "command to be discoverable by cmake."
    )
endif()

find_package(Doxygen)
if (NOT DOXYGEN_FOUND)
    message("Could not find Doxygen. API documentation will not be built.")
endif()
find_package(Python COMPONENTS Interpreter REQUIRED)