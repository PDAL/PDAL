#!/bin/bash

# Making a build directory for cmake - if we name it _build it will also be
# the output location for jupyterbook. This can be customizable if we want
mkdir -p doc/_build
cd doc/_build

cmake \
	-DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
	..

# Target makes dimension-table.csv, runs doxygen
cmake --build . --target doxygen
# Target creates conf.py
cmake --build . --target jupyterbook_config
