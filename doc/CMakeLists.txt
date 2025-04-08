find_package(Doxygen)
find_package(Python COMPONENTS Interpreter)
find_program(JUPYTERBOOK jupyter-book)
find_program(SPHINX_BUILD sphinx-build)

#### From GDAL
if ((NOT "${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
    AND DOXYGEN_FOUND)
    set(BUILD_API_DOCS_DEFAULT ON)
else()
    set(BUILD_API_DOCS_DEFAULT OFF)
endif()

#option(BUILD_API_DOCS "Build docs with Doxygen and Sphinx" ${BUILD_DOCS_DEFAULT})
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

# Output directory
set(BUILDDIR ${CMAKE_CURRENT_BINARY_DIR}/build)
# Build options to pass to Sphinx
set(SPHINX_BUILD_OPTS "-E -W --keep-going")

#set(PAPER "latex paper size to forward to sphinx (a4, letter)" "")

file(MAKE_DIRECTORY ${BUILDDIR})

set(ALL_SPHINX_OPTS "-d ${BUILDDIR}/doctrees ${SPHINX_BUILD_OPTS}")

# Configure python for sphinx here?

### 
### Doxygen setup
###

if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR "BUILD_API_DOCS=ON requires Doxygen")
endif()

# Path to doxyfile
set(DOXYFILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/doxygen.conf)

# read the doxyfile to a variable & replace strings if we want to build doxygen in
# the build directory, or give options to fail on warnings, etc -- not sure
file(STRINGS ${DOXYFILE_PATH} CONFIG_CONTENTS)
list(FILTER CONFIG_CONTENTS EXCLUDE REGEX "^#|^OUTPUT_DIRECTORY")
message("\"${CONFIG_CONTENTS}\"")

# set output & input directories?

set(DOXYGEN_BUILD_OPTS "")


###
### Doxygen generation
###

# always generate these dims w/ python for now.
set(PYTHON_DIMS_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/generate_dimensions.py)
add_custom_command(
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
    COMMAND ${Python_EXECUTABLE} ${PYTHON_DIMS_SCRIPT}
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
)
add_custom_target(dims
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/pydims.stamp
    COMMENT "Built Dimension Table CSV for Doxygen")

add_custom_command(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH} ${DOXYGEN_BUILD_OPTS}
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
)
# requiring the dimension generation script to get run before doxygen.
# readthedocs.yml does this currently, but not the docs makefile. Idk
add_custom_target(doxygen
    DEPENDS dims ${CMAKE_CURRENT_BINARY_DIR}/doxygen.stamp
    COMMENT "Built Doxygen")

###
### Jupyterbook generation
###

#!! trying to add this as a way to avoid appending to conf.py with `sed` / similar commands.
# writing an entire new cmake file just to call this command is silly but it works.

#!! Figure out where conf.py should be written to
set(JB_CONF_PATH "${CMAKE_SOURCE_DIR}/doc/conf.py")
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake 
    "set(JB_CONF_PATH \"${JB_CONF_PATH}\")\n"
    "file(READ \"$\{JB_CONF_PATH\}\" JB_CONF_CONTENTS)\n"
    "string(PREPEND JB_CONF_CONTENTS \"import os, sys; sys.path.append(os.path.abspath(\\\"./_ext\\\"))\\n\")\n"
    "file(WRITE \"$\{JB_CONF_PATH\}\" \"$\{JB_CONF_CONTENTS\}\")\n"
    )

function(jupyterbook_sphinx_config)
    set(JB_CONF_PATH "${CMAKE_SOURCE_DIR}/doc/conf.py")
    file(READ "${JB_CONF_PATH}" JB_CONF_CONTENTS)
    string(PREPEND JB_CONF_CONTENTS "import os, sys; sys.path.append(os.path.abspath(\"./_ext\"))\n")
    file(WRITE "${JB_CONF_PATH}" "${JB_CONF_CONTENTS}")
endfunction()

add_custom_target(call_jb_conf_prepend
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/prepend.cmake
    COMMENT "Prepended conf.py")

add_custom_command(
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    DEPENDS doxygen
    COMMAND ${JUPYTERBOOK} config sphinx doc/
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
)

add_custom_target(jupyterbook
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/jupyterbook.stamp
    COMMENT "Built Jupyterbook")

### Our final command to run @ build time
add_custom_target(generate_docs
    DEPENDS doxygen jupyterbook call_jb_conf_prepend
    COMMENT "Built Sphinx and Jupyterbook")
