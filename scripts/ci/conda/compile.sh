#!/bin/bash

pwd
ls

conda build recipe --clobber-file recipe/recipe_clobber.yaml
conda install recipe

pdal --version
