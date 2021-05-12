#!/bin/bash

mkdir packages

export CI_PLAT=""
if grep -q "windows" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="win"
fi

if grep -q "ubuntu" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="linux"
fi

if grep -q "macos" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="osx"
fi

conda build recipe --clobber-file recipe/recipe_clobber.yaml --output-folder packages -m ".ci_support/${CI_PLAT}_64_.yaml"
conda create -y -n test -c ./packages python=3.8 pdal
conda deactivate

conda activate test
pdal --version
conda deactivate
