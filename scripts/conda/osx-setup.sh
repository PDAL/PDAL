export CONDA_EXE=/Users/hobu/miniconda3/bin/conda

CONDA_ENV="pdal-build"

$CONDA_EXE env remove -n $CONDA_ENV -y
$CONDA_EXE config --add channels conda-forge
$CONDA_EXE create --name $CONDA_ENV -y
source /Users/hobu/miniconda3/bin/activate $CONDA_ENV
$CONDA_EXE install  -y laz-perf \
                laszip \
                libunwind \
                geotiff \
                sqlite \
                draco \
                libxml2 \
                nitro \
                curl \
                gdal \
                postgresql \
                hdf5 \
                cmake \
                compilers \
                libspatialite \
                eigen \
                ninja \
                libgdal \
                zstd \
                numpy \
                tiledb

activate $CONDA_ENV
