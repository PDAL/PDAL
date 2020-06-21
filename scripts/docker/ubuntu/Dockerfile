FROM pdal/ubuntubase:latest as builder
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
    cmake -G Ninja  \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_PLUGIN_CPD=ON \
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
    ctest -V

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    cd pdal/build  && \
    ninja install

RUN apt-get -y remove unzip bzip2 gcc-6 g++-6 ca-certificates && \
   apt-get -y autoremove

RUN . /opt/conda/etc/profile.d/conda.sh && \
    conda activate pdal && \
    conda remove -c conda-forge git cmake ninja conda-forge-ci-setup

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
        sudo && \
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
