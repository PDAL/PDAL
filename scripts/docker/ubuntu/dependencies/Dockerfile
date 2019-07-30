FROM ubuntu:bionic as builder
MAINTAINER Howard Butler <howard@hobu.co>

ARG PDAL_VERSION=master
ARG DESTDIR="/build"
ARG PARALLEL=2
ARG tiledb_version=1.4.1

ENV CC gcc
ENV CXX g++

RUN apt-get update -qq; \
    apt-get -qq remove postgis; \
    apt-get install -y --fix-missing --no-install-recommends \
        software-properties-common

RUN  apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --fix-missing --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        gfortran \
        git \
        libgeotiff-dev \
        libgeotiff-epsg \
        libarpack2-dev \
        libflann-dev \
        libhdf5-serial-dev \
        liblapack-dev \
        libtiff5-dev \
        openssh-client \
        python3-dev \
        python3-numpy \
        wget \
        automake \
        libtool \
        libspatialite-dev \
        libsqlite3-mod-spatialite \
        libhdf5-dev \
        libboost-filesystem1.65-dev \
        libboost-iostreams1.65-dev \
        libboost-program-options1.65-dev \
        libboost-system1.65-dev \
        libboost-thread1.65-dev \
        clang \
        libproj-dev \
        libc6-dev \
        libnetcdf-dev \
        libpng-dev \
        libjpeg-dev \
        libgif-dev \
        libwebp-dev \
        libhdf4-alt-dev \
        libhdf5-dev \
        unixodbc-dev \
        libsqlite3-dev \
        libltdl-dev \
        libcurl4-openssl-dev \
        libspatialite-dev \
        cython \
        python3-pip \
        time \
        ninja-build \
        python3-setuptools \
        unzip \
        libeigen3-dev \
        libxml2-dev \
        libssl-dev \
        liblzma-dev \
        libzstd1-dev \
        pkg-config \
        libgdal-dev \
        bash-completion \
        ninja-build \
        sqlite3 \
        libgeos-dev \
    libfreexl-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/LASzip/LASzip.git laszip \
    && cd laszip \
    && git checkout 3.4.1 \
    && cmake  \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_BUILD_TYPE="Release" \
     .  \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf laszip

RUN git clone  https://github.com/hobu/laz-perf.git \
    && cd laz-perf \
    && mkdir build \
    && cd build \
    && cmake .. \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf /laz-perf


RUN mkdir /nitro && cd /nitro \
    && git clone https://github.com/hobu/nitro \
    && cd nitro \
    && mkdir build \
    && cd build \
    && cmake ..\
        -G Ninja -DCMAKE_INSTALL_PREFIX=/usr/ \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf /nitro

RUN git clone --depth 1 --branch v0.4.6 https://github.com/gadomski/fgt.git \
    && cd fgt \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
        -G "Ninja" \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf /fgt

RUN git clone --depth 1 --branch v0.5.1 https://github.com/gadomski/cpd.git \
    && cd cpd \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DWITH_FGT=ON \
        -DCMAKE_SYSTEM_PREFIX_PATH="${DESTDIR}/usr" \
        -DCMAKE_CXX_FLAGS="-isystem ${DESTDIR}/usr/include" \
        -DWITH_STRICT_WARNINGS=OFF \
        -DWITH_DOCS=OFF \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
        -G "Ninja" \
    && ninja  -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install  \
    && rm -rf /cpd

RUN \
    curl -LO https://github.com/Kitware/CMake/releases/download/v3.14.5/cmake-3.14.5.tar.gz && tar zxvf cmake-3.14.5.tar.gz && cd cmake-3.14.5; \
    ./bootstrap --parallel=${PARALLEL} --prefix=/usr && make -j ${PARALLEL} && make install
