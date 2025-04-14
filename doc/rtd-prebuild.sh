#!/bin/bash

# Assuming readthedocs is running this from pdal root directory

# Making a build directory for cmake - if we name it _build it will also be
# the default output location for jupyterbook (I think?). This can be customizable if we want
mkdir -p doc/_build
cd doc/_build

# JB_CONF_APPEND variable replaces the sed command to append conf.py.
# JB_CONF_PREPEND replaces the sed command to prepend conf.py, set to
# "import os, sys; sys.path.append(os.path.abspath("./_ext"))" by default
cmake \
	 -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
	 -DBUILD_API_DOCS=ON \
	 -DJB_CONF_APPEND="[==[html_baseurl = os.environ.get(\"READTHEDOCS_CANONICAL_URL\", \"\")]==]" \
	 ..
# Target makes dimension-table.csv, runs doxygen
cmake --build . --target doxygen
# Target runs jupyter-book config
cmake --build . --target jupyterbook_config

