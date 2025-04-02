find_package(Doxygen)
find_program(JUPYTERBOOK jupyter-book)
find_program(SPHINX_BUILD sphinx-build)

#### From GDAL
if ((NOT "${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
    AND DOXYGEN_FOUND)
    set(BUILD_API_DOCS_DEFAULT ON)
else()
    set(BUILD_API_DOCS_DEFAULT OFF)
endif()

option(BUILD_API_DOCS "Build docs with Doxygen and Sphinx" ${BUILD_DOCS_DEFAULT})
#option(DOXYGEN_FAIL_ON_WARNINGS "Set to ON to fail the build if Doxygen produces warnings" OFF)

message(CMAKE_CURRENT_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
message(CMAKE_BINARY_DIR="${CMAKE_BINARY_DIR}")
message(CMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
message(CMAKE_SOURCE_DIR="${CMAKE_SOURCE_DIR}")

if (NOT SPHINX_BUILD)
    message(FATAL_ERROR "WITH_DOCS=ON requires sphinx-build")
endif()
if (NOT JUPYTERBOOK)
    message(FATAL_ERROR "WITH_DOCS=ON requires jupyter-book")
endif()

### 
### Sphinx setup
###

option(BUILDDIR "build directory" ${CMAKE_CURRENT_BINARY_DIR}/build)
option(SPHINX_BUILD_OPTS "Build options to pass to Sphinx" "-E -W --keep-going")
#option(PAPER "latex paper size to forward to sphinx (a4, letter)" "")

file(MAKE_DIRECTORY ${BUILDDIR})

#!! make a way to set workshop opts like in the makefile?
set(ALL_SPHINX_OPTS "-d ${BUILDDIR}/doctrees ${SPHINX_BUILD_OPTS}")

# Configure python for sphinx here?

if (BUILD_API_DOCS)
    ### 
    ### Doxygen config
    ###

    if (NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "BUILD_API_DOCS=ON requires Doxygen")
    endif()

    # read the doxyfile to a variable & replace strings if we want to build doxygen in
    # the build directory, or give options to fail on warnings, etc -- not sure
    file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/doxygen.conf CONFIG_CONTENTS)
    list(FILTER CONFIG_CONTENTS EXCLUDE REGEX "^#|^OUTPUT_DIRECTORY")
    message("\"${CONFIG_CONTENTS}\"")
    
    # set output & input directories?

    ###
    ### Doxygen generation
    ###

    # always generate these dims w/ python for now.
    set(PYTHON_DIMS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/generate_dimensions.py)
    add_custom_command(
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
        COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_DIMS_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
    )
    add_custom_target(dims
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
        COMMENT "Built Dimensions")

    add_custom_command(
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen/doxygen.conf
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp

    )
    # requiring the dimension generation script to get run before doxygen.
    # readthedocs.yml does this currently, but not the docs makefile. Idk
    add_custom_target(doxygen
        DEPENDS dims ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMENT "Built Doxygen")
endif()

###
### Jupyterbook generation
###

add_custom_command(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT $
)
