FROM pdal/ubuntu-dependencies builder
MAINTAINER Howard Butler <howard@hobu.co>

ARG PDAL_VERSION=master
ARG DESTDIR="/build"
ARG PARALLEL=2

ENV CC gcc
ENV CXX g++

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
