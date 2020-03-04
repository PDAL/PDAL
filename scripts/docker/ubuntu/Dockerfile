FROM pdal/ubuntubase:latest
MAINTAINER Andrew Bell <andrew@hobu.co>

SHELL ["/bin/bash", "-c"]
ENV CC gcc-6
ENV CXX g++-6

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    git clone http://github.com/PDAL/PDAL.git pdal

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    mkdir -p pdal/build && \
    cd pdal/build  && \
    LDFLAGS="$LDFLAGS -Wl,-rpath-link,$CONDA_PREFIX/lib" cmake -G Ninja  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DCMAKE_FIND_FRAMEWORK="NEVER" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_INSTALL_PREFIX=${CONDA_PREFIX} \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_HDF=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DBUILD_PGPOINTCLOUD_TESTS=OFF \
        -DWITH_LAZPERF=ON \
        -DWITH_ZSTD=ON \
        -DWITH_LASZIP=ON \
        ..

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    cd pdal/build  && \
    ninja

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    cd pdal/build  && \
    ctest
