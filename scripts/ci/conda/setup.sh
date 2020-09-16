#!/bin/bash

conda update -n base -c defaults conda
conda install conda-build -y
pwd
ls
git clone https://github.com/conda-forge/pdal-feedstock.git

cd pdal-feedstock
cat > recipe/recipe_clobber.yaml <<EOL
source:
  git_url: git@github.com:PDAL/PDAL.git
  git_rev: ${GITHUB_SHA}
  url:
  sha256:
EOL

ls recipe
