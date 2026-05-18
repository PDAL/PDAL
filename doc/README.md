# PDAL Documentation Building


## Setup Environment

```
mamba env create -n pdal-docs -f ./doc/environment.yml
mamba env update -n pdal-docs -f ./doc/local-build-environment.yml
mamba activate pdal-docs
```

## Build

```
cd doc
cmake -S . -B _build
cmake --build ./_build --target html
open _build/html
```
