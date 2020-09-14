#!/bin/bash

conda install -c conda-forge pdal --only-deps -y
conda install -c conda-forge cmake ninja compilers -y
mkdir build
