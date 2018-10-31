export CONDA_PREFIX=/Users/hobu/miniconda3/

export PATH=$CONDA_PREFIX/bin:$PATH

echo $PATH

conda remove pdal
conda activate pdal
conda config --add channels conda-forge
conda create --name pdal -y
conda install   -y laz-perf \
                laszip \
                libunwind \
                geotiff \
                jsoncpp \
                sqlite \
                libxml2 \
                nitro \
                curl \
                gdal=2.3.2 \
                postgresql \
                hdf5 \
                pcl \
                cmake \
                clang_osx-64 \
                clangxx_osx-64 \
                libspatialite \
                eigen \
                ninja \
                libgdal \
                geos \
                zstd \
                python=3.7 \
                numpy
#
BUILDDIR=conda-build

#export CC=/usr/bin/clang
#export CXX=/usr/bin/clang++
export CC=${CONDA_PREFIX}/bin/clang
export CXX=${CONDA_PREFIX}/bin/clang++
export GDAL_HOME=${CONDA_PREFIX}

#CONFIG="Unix Makefiles"
CONFIG="Ninja"

if ! [ -z "$1" ]; then
    CONFIG="$1"
fi


rm -rf $BUILDDIR
mkdir -p $BUILDDIR
cd $BUILDDIR
CC=$CC CXX=$CXX cmake   -G "$CONFIG"  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DBoost_DIR:FILEPATH="$CONDA_PREFIX/" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_PCL=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DWITH_LAZPERF=ON \
        -DWITH_ZSTD=ON \
        -DWITH_LASZIP=ON \
        ..


# if ! [ -z "Unix Makefiles" ]; then
#     make -j 4
# fi

if ! [ -z "Ninja" ]; then
    ninja
fi
