FROM ubuntu:bionic as builder
MAINTAINER Howard Butler <howard@hobu.co>

ARG PDAL_VERSION=master
ARG DESTDIR="/build"
ARG PARALLEL=16
ARG tiledb_version=1.4.1

ENV CC gcc
ENV CXX g++

RUN apt-get update -qq; \
    apt-get -qq remove postgis; \
    apt-get install -y --fix-missing --no-install-recommends \
        software-properties-common

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --fix-missing --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        gfortran \
        git \
        libarmadillo-dev \
        libarpack2-dev \
        libflann-dev \
        libhdf5-serial-dev \
        libhdf5-dev \
        libhdf5-cpp-100 \
        liblapack-dev \
        libtiff5-dev \
        openssh-client \
        python3-dev \
        python3-numpy \
        software-properties-common \
        wget \
        automake \
        libtool \
        libboost-filesystem1.65-dev \
        libboost-iostreams1.65-dev \
        libboost-program-options1.65-dev \
        libboost-system1.65-dev \
        libboost-thread1.65-dev \
        subversion \
        clang \
        clang-6.0 \
        libc6-dev \
        libopenjp2-7-dev \
        libpng-dev \
        libjpeg-dev \
        libgif-dev \
        libwebp-dev \
        libhdf4-alt-dev \
        libhdf5-dev \
        libpq-dev \
        libxerces-c-dev \
        unixodbc-dev \
        libsqlite3-dev \
        libmysqlclient-dev \
        libltdl-dev \
        libcurl4-openssl-dev \
        libssl-dev \
        python3-numpy \
        libdap-dev\
        cython \
        python3-pip \
        time \
        libhpdf-dev \
        python3-setuptools \
        libhpdf-dev \
        unzip \
        libzstd-dev \
        liblzma-dev \
        libeigen3-dev \
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


RUN git clone https://github.com/OSGeo/PROJ.git --branch 6.1 proj \
    && cd proj \
    && ./autogen.sh \
    && ./configure --prefix=/usr \
    && make -j ${PARALLEL} \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /proj


RUN git clone --branch master https://github.com/OSGeo/libgeotiff.git \
    &&    cd libgeotiff/libgeotiff \
    && ./autogen.sh \
    && ./configure --prefix=/usr --with-proj=/usr \
    && make -j ${PARALLEL} \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /libgeotiff

RUN git clone --branch release/3.0 https://github.com/OSGeo/gdal.git \
    &&    cd gdal/gdal \
    && ./configure --prefix=/usr \
            --mandir=/usr/share/man \
            --includedir=/usr/include/gdal \
            --with-threads \
            --with-grass=no \
            --with-hide-internal-symbols=yes \
            --with-rename-internal-libtiff-symbols=yes \
            --with-rename-internal-libgeotiff-symbols=yes \
            --with-libtiff=/usr/ \
            --with-geos=/usr/bin/geos-config \
            --with-geotiff=/usr \
            --with-proj=/usr \
            --with-ogdi=no \
            --with-curl \
            --with-pg \
            --with-ecw=no \
            --with-mrsid=no \
    && make -j ${PARALLEL} \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /gdal

RUN curl -LOs http://www.gaia-gis.it/gaia-sins/libspatialite-sources/libspatialite-5.0.0-beta0.tar.gz && tar zxvf libspatialite-5.0.0-beta0.tar.gz && cd /libspatialite-5.0.0-beta0 \
    && CXXFLAGS="-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H=1" CFLAGS="-DACCEPT_USE_OF_DEPRECATED_PROJ_API_H=1" ./configure --prefix=/usr \
    && make -j ${PARALLEL} \
    && make install \
    && DESTDIR=/ make install

RUN git clone --depth 1 --branch ${PDAL_VERSION} https://github.com/PDAL/PDAL.git pdal-git; \
    cd pdal-git;  \
    mkdir build;\
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
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
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_RIVLIB=OFF \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_TILEDB=OFF \
        -DENABLE_CTEST=OFF \
        -DWITH_LAZPERF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_ZSTD=ON \
        -DWITH_ZLIB=ON \
        -DWITH_TESTS=OFF \
        -DWITH_PDAL_JNI=OFF \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    ninja -j ${PARALLEL}; \
    ninja install; \
    DESTDIR=/ ninja install; \
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
    ninja -j ${PARALLEL}; \
    ninja install; \
    DESTDIR=/ ninja install; \
    rm -rf /libharu

RUN git clone https://github.com/PDAL/PRC \
    && cd PRC \
    && mkdir build \
    && cd build \
    && CXXFLAGS="-fno-stack-protector" CFLAGS="-fno-stack-protector" cmake \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DPDAL_DIR="/usr/lib/pdal/cmake" \
        -DCMAKE_BUILD_TYPE=Release  .. \
      -G "Ninja" \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf /PRC



RUN \
    git clone https://github.com/pubgeo/pubgeo.git \
    && cd pubgeo \
    && mkdir build \
    && cd build \
    && CXXFLAGS="-fno-stack-protector" CFLAGS="-fno-stack-protector" cmake \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DPDAL_DIR="/usr/lib/pdal/cmake" \
        -DCMAKE_BUILD_TYPE=Release  .. \
      -G "Ninja" \
    && ninja -j ${PARALLEL} \
    && ninja install \
    && DESTDIR=/ ninja install \
    && rm -rf /pubgeo

FROM ubuntu:bionic as runner

RUN apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
        sudo curl \
        vim unzip \
        unixodbc \
        libexpat1 \
        libxml2 \
        libwebp6 \
        netcdf-bin \
        libhdf4-0-alt \
        libgif7 \
        libdapclient6v5 \
        libtiff5 \
        libflann1.9 \
        libssl1.1 \
        zlib1g \
        libpng16-16 \
        python3-setuptools \
        liblzma5 \
        libzstd1 \
        libhdf5-cpp-100 \
        libfreexl1 \
        libgeos-c1v5 \
        libxerces-c3.2 \
        libopenjp2-7 \
        libpq5 \
        libhpdf-2.3.0 \
  && rm -rf /var/lib/apt/lists/*

RUN date

COPY --from=builder  /build/usr/bin/ /usr/bin/
COPY --from=builder  /build/usr/lib/ /usr/lib/
COPY --from=builder  /build/usr/include/ /usr/include/
COPY --from=builder  /build/usr/share/ /usr/share/

RUN \
    curl -LOs http://download.osgeo.org/proj/proj-datumgrid-1.8.zip &&  unzip -j -u -o proj-datumgrid-1.8.zip  -d /usr/share/proj && rm proj-datumgrid-1.8.zip ; \
    curl -LOs http://download.osgeo.org/proj/proj-datumgrid-europe-1.2.zip &&  unzip -j -u -o proj-datumgrid-europe-1.2.zip -d /usr/share/proj && rm proj-datumgrid-europe-1.2.zip ; \
    curl -LOs http://download.osgeo.org/proj/proj-datumgrid-oceania-1.0.zip &&  unzip -j -u -o proj-datumgrid-oceania-1.0.zip -d /usr/share/proj && rm proj-datumgrid-oceania-1.0.zip; \
    curl -LOs http://download.osgeo.org/proj/proj-datumgrid-world-1.0.zip &&  unzip -j -u -o proj-datumgrid-world-1.0.zip -d /usr/share/proj && rm proj-datumgrid-world-1.0.zip; \
    curl -LOs http://download.osgeo.org/proj/proj-datumgrid-north-america-1.2.zip &&  unzip -j -u -o proj-datumgrid-north-america-1.2.zip -d /usr/share/proj && rm proj-datumgrid-north-america-1.2.zip;
