FROM pdal/ubuntubase:latest as builder
MAINTAINER Andrew Bell <andrew@hobu.co>

SHELL ["conda", "run", "-n", "pdal", "/bin/bash", "-c"]

RUN git clone http://github.com/PDAL/PDAL.git pdal && \
    mkdir -p pdal/build && \
    cd pdal/build  && \
    LDFLAGS="-Wl,-rpath-link,$CONDA_PREFIX/lib" cmake -G Ninja  \
        -DCMAKE_LIBRARY_PATH:FILEPATH="$CONDA_PREFIX/lib" \
        -DCMAKE_INCLUDE_PATH:FILEPATH="$CONDA_PREFIX/include" \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_PLUGIN_CPD=OFF \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_HDF=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DBUILD_PLUGIN_E57=ON \
        -DBUILD_PGPOINTCLOUD_TESTS=OFF \
        -DWITH_LAZPERF=ON \
        -DWITH_ZSTD=ON \
        -DWITH_LASZIP=ON \
        ..

RUN cd pdal/build  && \
    ninja

RUN cd pdal/build  && \
    ctest -V

RUN cd pdal/build  && \
    ninja install

RUN conda remove -c conda-forge git cmake ninja conda-forge-ci-setup

RUN rm /opt/conda/envs/pdal/lib/*.a && \
    rm /opt/conda/envs/pdal/include/H5*.h && \
    rm /opt/conda/envs/pdal/include/h5*.mod && \
    rm /opt/conda/envs/pdal/include/tcl*.h && \
    rm /opt/conda/envs/pdal/include/itcl*.h && \
    rm /opt/conda/envs/pdal/include/tk*.h && \
    rm -rf /opt/conda/envs/pdal/include/spatialite && \
    rm -rf /opt/conda/envs/pdal/include/freetype2 && \
    rm -rf /opt/conda/envs/pdal/include/informix && \
    rm -rf /opt/conda/envs/pdal/include/fontconfig && \
    rm -rf /opt/conda/envs/pdal/include/X11 && \
    rm -rf /opt/conda/envs/pdal/lib/cmake

FROM ubuntu:bionic

ARG PDAL_CONDA=/opt/conda/envs/pdal

RUN apt-get update --fix-missing && \
    apt-get install -y \
        sudo ca-certificates && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/ /usr/
COPY --from=builder /lib/ /lib/
COPY --from=builder /${PDAL_CONDA}/lib/ /usr/lib/
COPY --from=builder /${PDAL_CONDA}/include/ /usr/include/
COPY --from=builder /${PDAL_CONDA}/share/ /usr/share/

ENV GDAL_DATA=/usr/share/gdal
ENV PDAL_DRIVER_PATH=/usr/lib
ENV PROJ_LIB=/usr/share/proj
