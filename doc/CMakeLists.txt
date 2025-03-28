find_package(Doxygen QUIET)
## jupyterbook?
find_program(SPHINX_BUILD sphinx-build)

#### From GDAL
if ((NOT "${CMAKE_BINARY_DIR}" STREQUAL "${CMAKE_SOURCE_DIR}")
    AND DOXYGEN_FOUND AND SPHINX_BUILD)
    set(BUILD_DOCS_DEFAULT ON)
else()
    set(BUILD_DOCS_DEFAULT OFF)
endif()

option(BUILD_DOCS "Build docs with Doxygen and Sphinx" ${BUILD_DOCS_DEFAULT})

option(DOXYGEN_FAIL_ON_WARNINGS "Set to ON to fail the build if Doxygen produces warnings" OFF)
if (BUILD_DOCS)
    #### Sphinx setup ####

    # build or _build? jupyterbook does '_build' but we set it to 'build' in the makefile
    option(BUILDDIR "build directory" ${CMAKE_CURRENT_SOURCE_DIR}/build)
    option(SPHINX_BUILD_OPTS "Build options to pass to Sphinx" "-E -W --keep-going")
    #option(PAPER "latex paper size to forward to sphinx (a4, letter)" "")

    file(MAKE_DIRECTORY ${SPHINX_BUILD_DIR})
    file(MAKE_DIRECTORY ${SPHINX_BUILD_DIR}/html_extra)

    # make a way to set workshop opts
    set(ALL_SPHINX_OPTS "-d ${BUILDDIR}/doctrees ${SPHINX_BUILD_OPTS}")

    # Configure python for sphinx here?


endif()


