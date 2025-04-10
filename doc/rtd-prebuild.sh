#!/bin/bash

# Assuming readthedocs is running this from pdal root directory

# Making a build directory for cmake - if we name it _build it will also be
# the default output location for jupyterbook. This can be customizable if we want
mkdir -p _build
cd _build

# JB_CONF_APPEND variable replaces the sed command to append conf.py.
# JB_CONF_PREPEND replaces the sed command to prepend conf.py, set to
# "import os, sys; sys.path.append(os.path.abspath("./_ext"))" by default
cmake \
	 -DBUILD_API_DOCS=ON \
	 -DJB_CONF_APPEND="html_baseurl = os.environ.get(\\\"READTHEDOCS_CANONICAL_URL\\\", \\\"\\\")" \
	 ../doc/
# Target makes dimension-table.csv, runs doxygen, runs jupyter-book config
cmake --build . --target generate_docs

