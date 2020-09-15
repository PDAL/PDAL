#!/bin/bash

conda update -n base -c defaults conda
conda install pdal --only-deps -y
conda install cmake ninja compilers -y
mkdir build
