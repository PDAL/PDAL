#!/bin/bash

conda activate test
echo $PATH
pwd
where python
where cmake
conda install pdal --only-deps -y
conda install cmake ninja compilers -y
mkdir build
