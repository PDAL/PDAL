cmake_minimum_required(VERSION 3.13)
# This allows us to build standalone without PDAL dependencies
if(NOT ROOT_DIR)
    project(pdal-docs)
    # Could also do ${CMAKE_CURRENT_LIST_DIR}/..
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
    # If we're building from the doc directory, set the sphinx build dir as our current build dir.
    set(SPHINX_BUILDDIR ${CMAKE_CURRENT_BINARY_DIR})
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
    message(FATAL_ERROR "Building documentation requires sphinx-build")
endif()
if (NOT JUPYTERBOOK)
    message(FATAL_ERROR "Building documentation requires jupyter-book")
endif()

# From GDAL. I feel like this probably doesn't need to be here.
if ((NOT "${CMAKE_BINARY_DIR}" STREQUAL "${ROOT_DIR}")
    AND DOXYGEN_FOUND)
    set(BUILD_API_DOCS_DEFAULT ON)
else()
    set(BUILD_API_DOCS_DEFAULT OFF)
endif()

option(BUILD_API_DOCS "Build API documentation with Doxygen" ${BUILD_DOCS_DEFAULT})
#option(DOXYGEN_FAIL_ON_WARNINGS "Set to ON to fail the build if Doxygen produces warnings" OFF)

### 
### Sphinx & Jupyter Book setup
###

# Builder options -- can set "--builder <builder>" for sphinx or jupyter-book targets
set(BUILDER "" CACHE STRING "Sphinx builder to use, e.g. html (--builder=<builder>)")

# Output directory - if we're building standalone, this already got created.
# If we're building from pdal source, specify our own.
if (NOT SPHINX_BUILDDIR)
    set(SPHINX_BUILDDIR ${PDAL_DOC_DIR}/_build)
    file(MAKE_DIRECTORY ${SPHINX_BUILDDIR})
endif()

#!! Unused for now
set(JB_BUILD_OPTS "--output-dir ${SPHINX_BUILDDIR}")

if (BUILD_API_DOCS)

    ### 
    ### Doxygen setup
    ###

    if (NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "BUILD_API_DOCS=ON requires Doxygen")
    endif()

    # Path to doxygen config file
    set(DOXYFILE_PATH ${PDAL_DOC_DIR}/doxygen/doxygen.conf)
    # Tried to default to 1> /dev/null but that isn't working
    set(DOXYGEN_BUILD_OPTS "" CACHE STRING "Additional options to pass to Doxygen")

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
### Jupyterbook config
###

add_custom_command(
    WORKING_DIRECTORY ${ROOT_DIR}
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    COMMAND ${JUPYTERBOOK} config sphinx doc/
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
)

add_custom_target(jupyterbook_config
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    COMMENT "Configured jupyter-book")
    
set(JB_CONF_PATH "${PDAL_DOC_DIR}/conf.py")

# Added this as a way to avoid appending to conf.py with sed or similar commands.
# writing an entire new cmake file just to add one line is silly, but it works.

# Right now you need to double-escape the quotes and newlines for the strings you're adding.
# Maybe there's a better way to do this?

set(JB_CONF_PREPEND "import os, sys; sys.path.append(os.path.abspath(\\\"./_ext\\\"))\\n" 
    CACHE STRING "Lines to prepend to conf.py.")
set(JB_CONF_APPEND "" 
    CACHE STRING "Lines to append to conf.py.")

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake 
    "file(READ \"${JB_CONF_PATH}\" JB_CONF_CONTENTS)\n"
    "string(PREPEND JB_CONF_CONTENTS \"${JB_CONF_PREPEND}\")\n"
    "string(APPEND JB_CONF_CONTENTS \"${JB_CONF_APPEND}\")\n"
    "file(WRITE \"${JB_CONF_PATH}\" \"$\{JB_CONF_CONTENTS\}\")\n"
    )

add_custom_target(modify_jb_conf
    DEPENDS jupyterbook_config
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake)

###
### Sphinx build setup
###

# Build options to pass to Sphinx
set(SPHINX_BUILD_OPTS "-E -W --keep-going" CACHE STRING "Additional options to pass to Sphinx")
#set(PAPER "latex paper size to forward to sphinx (a4, letter)" "")
# Sphinx options
set(ALL_SPHINX_OPTS "-d ${SPHINX_BUILDDIR}/doctrees ${SPHINX_BUILD_OPTS}")

###
### Final targets
###

add_custom_target(generate_docs
    DEPENDS jupyterbook_config modify_jb_conf
    COMMENT "Configured Doxygen and jupyter-book")

# From the docs makefile - pretty much does the same thing as the jupyter-book
# target. Probably not needed here
add_custom_target(sphinx-build
    WORKING_DIRECTORY ${PDAL_DOC_DIR}
    DEPENDS generate_docs
    COMMAND ${SPHINX_BUILD} ${BUILDER} ${ALL_SPHINX_OPTS} ${PDAL_DOC_DIR} ${SPHINX_BUILDDIR}/html)

add_custom_target(jupyter-book
    WORKING_DIRECTORY ${ROOT_DIR}
    COMMAND ${JUPYTERBOOK} build ${BUILDER} doc/
    COMMENT "Built Jupyter Book")