#!/bin/bash

conda install -c conda-forge conda-build boa -y
pwd
ls
git clone https://github.com/conda-forge/libpdal-feedstock.git

cd libpdal-feedstock

cat > recipe/recipe_clobber.yaml <<EOL
source:
  path: ../../
  url:
  sha256:
  patches:

build:
  number: 2112
EOL

ls recipe
