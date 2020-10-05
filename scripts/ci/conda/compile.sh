#!/bin/bash

mkdir packages

conda build recipe --clobber-file recipe/recipe_clobber.yaml --output-folder packages
conda install -c ./packages pdal

pdal --version
