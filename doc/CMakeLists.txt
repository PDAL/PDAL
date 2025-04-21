cmake_minimum_required(VERSION 3.13)
# This allows us to build standalone without PDAL dependencies
if(NOT ROOT_DIR)
    project(pdal-docs)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
    set(PDAL_CMAKE_DIR ${ROOT_DIR}/cmake)
    # If we're building from the doc directory, set the sphinx build dir as our current build dir.
    set(SPHINX_BUILDDIR ${CMAKE_CURRENT_BINARY_DIR})
endif()

set(PDAL_DOC_DIR ${ROOT_DIR}/doc)

###
### Dependencies
###

include(${PDAL_CMAKE_DIR}/docs.cmake)

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

if (DOXYGEN_FOUND)
    ### 
    ### Doxygen setup
    ###

    # Path to doxygen config file
    set(DOXYFILE_PATH ${PDAL_DOC_DIR}/doxygen/doxygen.conf)
    set(DOXYGEN_BUILD_OPTS "" CACHE STRING "Additional options to pass to Doxygen")

    ###
    ### Doxygen generation
    ###

    # Always generate these dims w/ python for now.
    set(PYTHON_DIMS_SCRIPT ${PDAL_DOC_DIR}/generate_dimensions.py)
    add_custom_command(
        WORKING_DIRECTORY ${PDAL_DOC_DIR}
        OUTPUT ${PDAL_DOC_DIR}/dimension_table.csv
        COMMAND ${Python_EXECUTABLE} ${PYTHON_DIMS_SCRIPT}
    )
    add_custom_command(
        WORKING_DIRECTORY ${ROOT_DIR}
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH} ${DOXYGEN_BUILD_OPTS}
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
    )
    # requiring the dimension generation script to get run before doxygen.
    # readthedocs.yml does this currently, but not the docs makefile. Idk
    add_custom_target(doxygen
        DEPENDS ${PDAL_DOC_DIR}/dimension_table.csv ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMENT "Built Doxygen")

endif()

###
### Jupyterbook config
###

set(JB_CONF_FILE "${PDAL_DOC_DIR}/conf.py")

add_custom_command(
    WORKING_DIRECTORY ${ROOT_DIR}
    OUTPUT ${JB_CONF_FILE}
    COMMAND ${JUPYTERBOOK} config sphinx doc/
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
)

# Added this as a way to avoid appending to conf.py with sed or similar commands.
# writing an entire new cmake file just to add one line is silly, but it works.

set(JB_CONF_PREPEND "[==[import os, sys; sys.path.append(os.path.abspath(\"./_ext\"))\n]==]"
    CACHE STRING "Lines to prepend to conf.py.")
set(JB_CONF_APPEND "" 
    CACHE STRING "Lines to append to conf.py.")

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake 
    "file(READ \"${JB_CONF_FILE}\" JB_CONF_CONTENTS)\n"
    "string(PREPEND JB_CONF_CONTENTS ${JB_CONF_PREPEND})\n"
    "string(APPEND JB_CONF_CONTENTS ${JB_CONF_APPEND})\n"
    "file(WRITE \"${JB_CONF_FILE}\" \"$\{JB_CONF_CONTENTS\}\")"
)

add_custom_target(jupyterbook_config
    DEPENDS ${JB_CONF_FILE}
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake
    COMMENT "Configured sphinx and jupyter-book")

###
### Final targets
###

add_custom_target(jupyter-book
    WORKING_DIRECTORY ${ROOT_DIR}
    COMMAND ${JUPYTERBOOK} build ${BUILDER} doc/
    COMMENT "Built Jupyter Book")