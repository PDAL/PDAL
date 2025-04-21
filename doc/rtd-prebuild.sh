#!/bin/bash

# Assuming readthedocs is running this from pdal root directory

# Making a build directory for cmake - if we name it _build it will also be
# the default output location for jupyterbook (I think?). This can be customizable if we want
mkdir -p doc/_build
cd doc/_build

cmake \
	-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
	..
# Target makes dimension-table.csv, runs doxygen
cmake --build . --target doxygen
# Target runs jupyter-book config
cmake --build . --target jupyterbook_config
