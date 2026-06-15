#!/bin/bash

conda info
conda list
conda config --show-sources

rm -f ~/.condarc

# For Python 3.13
conda config --add channels conda-forge/label/python_rc

conda config --show-sources

conda config --show

conda install -c conda-forge rattler-build yq -y

git clone  https://github.com/conda-forge/libpdal-feedstock.git

cd libpdal-feedstock

# Patch version: "X.Y.Z" to "X.Y.99"
sed -E 's/version: "([0-9]+)\.([0-9]+)\.([0-9]+)"/version: "\1.\2.99"/' < recipe/recipe.yaml > recipe.yaml
mv recipe.yaml recipe/recipe.yaml

yq -y -i '.source.url = ""' recipe/recipe.yaml
yq -y -i '.source.sha256 = ""' recipe/recipe.yaml
yq -y -i '.source.path = "../../"' recipe/recipe.yaml
yq -y -i '.build.number = 2112' recipe/recipe.yaml

# Update installation messages of plugins to reflect the appropriate channel
sed "s/-c conda-forge/-c pdal-master/" < recipe/build_core.sh >  recipe/build.sh.new
mv recipe/build.sh.new  recipe/build.sh
sed "s/-c conda-forge/-c pdal-master/" < recipe/build_core.bat > recipe/build.bat.new
mv recipe/build.bat.new  recipe/build.bat


ls recipe
