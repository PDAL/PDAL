cmake_minimum_required(VERSION 3.13)
# This allows us to build standalone without PDAL dependencies
if(NOT ROOT_DIR)
    project(pdal-docs)
    set(ROOT_DIR "${PROJECT_SOURCE_DIR}/..")
    set(PDAL_CMAKE_DIR ${ROOT_DIR}/cmake)
endif()

set(PDAL_DOC_DIR ${ROOT_DIR}/doc)

###
### Dependencies
###

include(${PDAL_CMAKE_DIR}/docs.cmake)

### 
### Sphinx & Jupyter Book setup
###

set(CUSTOM_SPHINX_BUILDER "" CACHE STRING "Custom Sphinx builder to pass to jupyter-book")
set(JB_BUILD_LOCATION "${PDAL_DOC_DIR}/" CACHE STRING "Directory to create jupyter-book `_build` directory in")

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
        DEPENDS ${PDAL_DOC_DIR}/dimension_table.csv
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH} ${DOXYGEN_BUILD_OPTS}
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
    )
    add_custom_target(doxygen
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
        COMMENT "Building Doxygen")

endif()

###
### Logo assets (optional, maintainer-only)
###
### The PDAL logos are maintained as SVG (doc/_static/logo); the PNG
### derivatives the docs reference (e.g. latex_logo, since pdflatex has no
### native SVG support) are committed to the repo like any other doc
### asset, so building the docs never needs an SVG rasterizer. This
### target is only a convenience for regenerating those PNGs by hand
### after editing a master SVG -- it is not a dependency of any build
### target below, and doc/generate_logo_assets.sh no-ops cleanly if
### rsvg-convert isn't installed.
if (RSVG_CONVERT_EXECUTABLE)
    add_custom_target(logo_assets
        COMMAND ${PDAL_DOC_DIR}/generate_logo_assets.sh
        WORKING_DIRECTORY ${PDAL_DOC_DIR}
        COMMENT "Regenerating PNG logo assets from the master SVGs (optional, maintainer-only)")
endif()

###
### Jupyterbook config
###

if(SED_EXECUTABLE)
    set(JB_CONF_FILE "${PDAL_DOC_DIR}/conf.py")

    # If we aren't using the sphinx-build command, we don't need to configure
    # Jupyter Book -- everything will be created from _config.yml.
    add_custom_command(
        WORKING_DIRECTORY ${ROOT_DIR}
        OUTPUT ${JB_CONF_FILE}
        COMMAND ${JUPYTERBOOK} config sphinx doc/
        )

    set(PREPEND_STRING [==[1i\import os, sys\; sys.path.append(os.path.abspath(\"./_ext\"))]==])

    add_custom_target(jupyterbook_config
        WORKING_DIRECTORY ${ROOT_DIR}
        DEPENDS ${JB_CONF_FILE}
        COMMAND ${SED_EXECUTABLE} -i \"${PREPEND_STRING}\" ${JB_CONF_FILE}
        COMMENT "Configuring Jupyter Book for Sphinx")
endif()

###
### Final targets
###

set(BUILDERS 
    "html" 
    "singlehtml" 
    "pdflatex" 
    "latex" 
    "dirhtml" 
    "pdfhtml" 
    "linkcheck" 
    "custom")

foreach(BUILDER ${BUILDERS})
    if(${BUILDER} STREQUAL "custom" AND CUSTOM_SPHINX_BUILDER)
        add_custom_target(${CUSTOM_SPHINX_BUILDER}
            WORKING_DIRECTORY ${ROOT_DIR}
            COMMAND 
                ${JUPYTERBOOK} build 
                    --builder ${BUILDER} 
                    --custom-builder ${CUSTOM_SPHINX_BUILDER} 
                    --path-output ${JB_BUILD_LOCATION} 
                    doc/
            COMMENT "Building Jupyter Book ${CUSTOM_SPHINX_BUILDER}")
    else()
        add_custom_target(${BUILDER}
            WORKING_DIRECTORY ${ROOT_DIR}
            COMMAND ${JUPYTERBOOK} build 
                --builder ${BUILDER} 
                --path-output ${JB_BUILD_LOCATION} 
                doc/
            COMMENT "Building Jupyter Book ${BUILDER}")
    endif()
endforeach()
