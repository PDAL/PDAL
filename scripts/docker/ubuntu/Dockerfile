FROM ubuntu:bionic as builder
MAINTAINER Howard Butler <howard@hobu.co>

ARG PDAL_VERSION=master
ARG DESTDIR="/build"
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
        libarpack2-dev \
        libflann-dev \
        libhdf5-serial-dev \
        liblapack-dev \
        libtiff5-dev \
        openssh-client \
        python-dev \
        python-numpy \
        wget \
        automake \
        libtool \
        libspatialite-dev \
        libsqlite3-mod-spatialite \
        libhdf5-dev \
        libjsoncpp-dev \
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
        python-pip \
        time \
        ninja-build \
        python-setuptools \
        unzip \
        libeigen3-dev \
        libxml2-dev \
        libssl-dev \
        liblzma-dev \
        libzstd1-dev \
        pkg-config \
        libgdal-dev \
        bash-completion

RUN    git clone https://github.com/LASzip/LASzip.git laszip; \
    cd laszip; \
    git checkout 3.1.1; \
    cmake  \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_BUILD_TYPE="Release" \
     . ; \
    ninja ; \
    ninja install; \
    rm -rf laszip

RUN git clone  https://github.com/hobu/laz-perf.git; \
    cd laz-perf; \
    mkdir build; \
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
    ; \
    ninja; \
    ninja install; \
    rm -rf /laz-perf


RUN mkdir /nitro; cd /nitro; \
    git clone https://github.com/hobu/nitro; \
    cd nitro; \
    mkdir build; \
    cd build; \
    cmake ..\
        -G Ninja -DCMAKE_INSTALL_PREFIX=/usr/ \
    ; \
    ninja; \
    ninja install \
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
    && ninja \
    && ninja install \
    && rm -rf /fgt

RUN git clone --depth 1 --branch v0.5.1 https://github.com/gadomski/cpd.git \
    && cd cpd \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DWITH_JSONCPP=OFF \
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
    && ninja  \
    && ninja install \
    && rm -rf /cpd


RUN git clone --depth 1 --branch ${PDAL_VERSION} https://github.com/PDAL/PDAL.git pdal-git; \
    cd pdal-git;  \
    mkdir build;\
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_PREFIX_PATH:FILEPATH="${DESTDIR}" \
        -DCMAKE_SYSTEM_PREFIX_PATH="${DESTDIR}/usr" \
        -DCMAKE_CXX_FLAGS="-isystem ${DESTDIR}/usr/include" \
        -DBUILD_PLUGIN_CPD=ON \
        -DBUILD_PLUGIN_MBIO=OFF \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_I3S=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_MRSID=OFF \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_OCI=OFF \
        -DBUILD_PLUGIN_PCL=OFF \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_RIVLIB=OFF \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_TILEDB=ON \
        -DENABLE_CTEST=OFF \
        -DWITH_LAZPERF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_ZSTD=ON \
        -DWITH_ZLIB=ON \
        -DWITH_TESTS=OFF \
        -DWITH_PDAL_JNI=OFF \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    ninja; \
    ninja install; \
    rm -rf /pdal-git

#
# Haru hasn't been updated for years.  This tag is the head.
#
RUN git clone https://github.com/libharu/libharu \
    && cd libharu \
    && git checkout d84867ebf9f3de6afd661d2cdaff102457fbc371 \
    && mkdir build \
    && cd build \
    && cmake \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_PREFIX_PATH:FILEPATH="${DESTDIR}" \
        -DCMAKE_SYSTEM_PREFIX_PATH="${DESTDIR}/usr" \
        -DCMAKE_CXX_FLAGS="-isystem ${DESTDIR}/usr/include" \
      ..; \
    ninja; \
    ninja install; \
    rm -rf /libharu

RUN git clone https://github.com/PDAL/PRC \
    && cd PRC \
    && mkdir build \
    && cd build \
    && CXXFLAGS="-fno-stack-protector" CFLAGS="-fno-stack-protector" cmake \
        -DCMAKE_PREFIX_PATH:FILEPATH="${DESTDIR}" \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_SYSTEM_PREFIX_PATH="${DESTDIR}/usr" \
        -DCMAKE_CXX_FLAGS="-isystem ${DESTDIR}/usr/include -L${DESTDIR}/usr/lib" \
        -DPDAL_DIR="${DESTDIR}/usr/lib/pdal/cmake" \
        -DCMAKE_BUILD_TYPE=Release  .. \
      -G "Ninja" \
    && ninja \
    && ninja install \
    && rm -rf /PRC

RUN \
    wget https://github.com/TileDB-Inc/TileDB/archive/${tiledb_version}.tar.gz; \
    tar xzf ${tiledb_version}.tar.gz; \
    rm ${tiledb_version}.tar.gz; \
    cd TileDB-${tiledb_version}; \
    mkdir build; \
    cd build; \
    ../bootstrap --prefix=/usr --enable-s3; \
    make -j2; \
    make install-tiledb; \
    cd /; \
    rm -rf TileDB-${tiledb_version};

FROM ubuntu:bionic as runner

RUN apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        sudo curl \
        vim unzip \
        unixodbc \
        libgeotiff2 \
        libgeotiff-epsg \
        libexpat1 \
        libxml2 \
        libjsoncpp1 \
        libwebp6 \
        netcdf-bin \
        libhdf4-0-alt \
        libgif7 \
        libdapclient6v5 \
        libspatialite7 \
        libsqlite3-mod-spatialite \
        spatialite-bin \
        libtiff5 \
        libflann1.9 \
        libssl1.1 \
        libpython2.7 \
        libproj-dev \
        proj-bin \
        gdal-bin \
        proj-data \
        zlib1g \
        libpng16-16 \
        python-pip \
        python-setuptools \
        liblzma5 \
        libzstd1 \
        libhdf5-cpp-100 ;\
  rm -rf /var/lib/apt/lists/*

RUN date

COPY --from=builder  /build/usr/bin/ /usr/bin/
COPY --from=builder  /build/usr/lib/ /usr/lib/
COPY --from=builder  /build/usr/include/ /usr/include/

RUN \
    curl -LOs http://download.osgeo.org/proj/vdatum/usa_geoid2012.zip && unzip -j -u usa_geoid2012.zip -d /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/usa_geoid2009.zip && unzip -j -u usa_geoid2009.zip -d /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/usa_geoid2003.zip && unzip -j -u usa_geoid2003.zip -d /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/usa_geoid1999.zip && unzip -j -u usa_geoid1999.zip -d /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/vertcon/vertconc.gtx && mv vertconc.gtx /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/vertcon/vertcone.gtx && mv vertcone.gtx /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/vertcon/vertconw.gtx && mv vertconw.gtx /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/egm96_15/egm96_15.gtx && mv egm96_15.gtx /usr/share/proj; \
    curl -LOs http://download.osgeo.org/proj/vdatum/egm08_25/egm08_25.gtx && mv egm08_25.gtx /usr/share/proj

