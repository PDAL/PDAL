# should make sure we can build standalone. Otherwise we require all of PDAL's dependencies
if(NOT ROOT_DIR)
    project(pdal-docs)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
endif()

set(PDAL_DOC_DIR ${ROOT_DIR}/doc)

###
### Dependencies
###

find_package(Doxygen)
find_package(Python COMPONENTS Interpreter REQUIRED)
find_program(JUPYTERBOOK jupyter-book)
find_program(SPHINX_BUILD sphinx-build)

if (NOT SPHINX_BUILD)
    message(FATAL_ERROR "WITH_DOCS=ON requires sphinx-build")
endif()
if (NOT JUPYTERBOOK)
    message(FATAL_ERROR "WITH_DOCS=ON requires jupyter-book")
endif()

#### From GDAL. I feel like this probably doesn't need to be here.
if ((NOT "${CMAKE_BINARY_DIR}" STREQUAL "${ROOT_DIR}")
    AND DOXYGEN_FOUND)
    set(BUILD_API_DOCS_DEFAULT ON)
else()
    set(BUILD_API_DOCS_DEFAULT OFF)
endif()

option(BUILD_API_DOCS "Build API documentation with Doxygen" ${BUILD_DOCS_DEFAULT})
#option(DOXYGEN_FAIL_ON_WARNINGS "Set to ON to fail the build if Doxygen produces warnings" OFF)

### 
### Sphinx setup
###

#!! if we just do `jb build doc/` it doesn't use any of these options.
# not sure how important they are.

# Output directory - defaults to the doc source directory
# if we build from the docs dir this will be screwed up
set(BUILDDIR ${PDAL_DOC_DIR}/_build_)
# Build options to pass to Sphinx
set(SPHINX_BUILD_OPTS "-E -W --keep-going")
#set(PAPER "latex paper size to forward to sphinx (a4, letter)" "")
# Sphinx options -- run relative from the source doc directory, if it matters
set(ALL_SPHINX_OPTS "-d ${BUILDDIR}/doctrees ${SPHINX_BUILD_OPTS}")

# Create the build directory our final output will be written to
#!! `jb build doc/` doesn't use this, makes default `_build` dir instead
file(MAKE_DIRECTORY ${BUILDDIR})

if (BUILD_API_DOCS)

    ### 
    ### Doxygen setup
    ###

    if (NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "BUILD_API_DOCS=ON requires Doxygen")
    endif()

    # Path to doxygen config file
    set(DOXYFILE_PATH ${PDAL_DOC_DIR}/doxygen/doxygen.conf)
    set(DOXYGEN_BUILD_OPTS "1> /dev/null")

    #!! could read the doxyfile to a variable & replace strings if we want to build doxygen in
    # the main build directory, or give options to fail on warnings, etc -- not sure
    #file(STRINGS ${DOXYFILE_PATH} CONFIG_CONTENTS)
    #list(FILTER CONFIG_CONTENTS EXCLUDE REGEX "^#|^OUTPUT_DIRECTORY")


    ###
    ### Doxygen generation
    ###

    # Always generate these dims w/ python for now.
    set(PYTHON_DIMS_SCRIPT ${PDAL_DOC_DIR}/generate_dimensions.py)
    add_custom_command(
        WORKING_DIRECTORY ${PDAL_DOC_DIR}
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
        COMMAND ${Python_EXECUTABLE} ${PYTHON_DIMS_SCRIPT}
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
    )
    add_custom_target(dims
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
        COMMENT "Built Dimension Table CSV for Doxygen")

    add_custom_command(
        WORKING_DIRECTORY ${ROOT_DIR}
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH} ${DOXYGEN_BUILD_OPTS}
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

# Added this as a way to avoid appending to conf.py with `sed` or similar commands.
# writing an entire new cmake file just to add one line is silly, but it works.

#!! Figure out where conf.py should be written to
set(JB_CONF_PATH "${PDAL_DOC_DIR}/conf.py")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake 
    "set(JB_CONF_PATH \"${JB_CONF_PATH}\")\n"
    "file(READ \"$\{JB_CONF_PATH\}\" JB_CONF_CONTENTS)\n"
    "string(PREPEND JB_CONF_CONTENTS \"import os, sys; sys.path.append(os.path.abspath(\\\"./_ext\\\"))\\n\")\n"
    "file(WRITE \"$\{JB_CONF_PATH\}\" \"$\{JB_CONF_CONTENTS\}\")\n"
    )

add_custom_target(call_jb_conf_prepend
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake)

add_custom_command(
    WORKING_DIRECTORY ${ROOT_DIR}
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    DEPENDS doxygen
    COMMAND ${JUPYTERBOOK} config sphinx doc/
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
)

add_custom_target(jupyterbook
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    COMMENT "Built Jupyterbook")

###
### Our final commands to run @ build time
###

add_custom_target(generate_docs
    DEPENDS doxygen jupyterbook call_jb_conf_prepend
    COMMENT "Configured Doxygen and Jupyterbook")

#!! not sure which one to use
add_custom_target(html
    WORKING_DIRECTORY ${PDAL_DOC_DIR}
    DEPENDS generate_docs
    COMMAND ${SPHINX_BUILD} -b html ${ALL_SPHINX_OPTS} ${PDAL_DOC_DIR} ${BUILDDIR}/html)
add_custom_target(jb
    WORKING_DIRECTORY ${ROOT_DIR}
    COMMAND ${JUPYTERBOOK} build doc/)