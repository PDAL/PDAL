#!/bin/bash

# Store existing PDAL env vars and set to this conda env
# so other PDAL installs don't pollute the environment

if [[ -n "$PDAL_DRIVER_PATH" ]]; then
    export _CONDA_SET_PDAL_DRIVER_PATH=$PDAL_DRIVER_PATH
fi

export PDAL_DRIVER_PATH=$CONDA_PREFIX/lib

# Support plugins if the plugin directory exists
# i.e if it has been manually created by the user
if [[ ! -d "$PDAL_DRIVER_PATH" ]]; then
    unset PDAL_DRIVER_PATH
fi

