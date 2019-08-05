FROM centos:7.6.1810 as builder
MAINTAINER Howard Butler <howard@hobu.co>

ARG DESTDIR="/build"
ARG PDAL_VERSION=hobu-catchup


RUN yum update -y && yum groupinstall 'Development Tools' -y &&\
    yum install epel-release -y && \
    yum install centos-release-scl -y && \
    yum install devtoolset-7 -y && \
    yum install -y  \
        wget \
        cmake3 \
        ninja-build \
        zlib-devel \
        python34-devel \
        libxml2-devel \
        curl-devel \
        python34-pip \
        sqlite-devel \
        libtiff-devel \
        geos-devel


SHELL [ "/usr/bin/scl", "enable", "devtoolset-7"]

RUN gcc --version

RUN git clone --branch 6.0 https://github.com/OSGeo/proj.4.git \
    && cd proj.4 \
    && ./autogen.sh \
    && ./configure --prefix=/usr \
    && make -j 4 \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /proj.4

RUN git clone --branch master https://github.com/OSGeo/libgeotiff.git \
    &&    cd libgeotiff/libgeotiff \
    && ./autogen.sh \
    && ./configure --prefix=/usr --with-proj=/build/usr \
    && make -j 4 \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /libgeotiff

RUN git clone --branch master https://github.com/OSGeo/gdal.git \
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
            --with-geotiff=/build/usr \
            --with-proj=/build/usr \
            --with-geos \
            --with-curl \
            --with-pg \
            --with-ecw=no \
            --with-mrsid=no \
    && make -j 4 \
    && make install \
    && DESTDIR=/ make install \
    && rm -rf /gdal
#
RUN ln -s /usr/bin/cmake3 /usr/bin/cmake




RUN git clone https://github.com/LASzip/LASzip.git laszip; \
    cd laszip; \
    git checkout 3.2.9; \
    cmake  \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_BUILD_TYPE="Release" \
     . ; \
    ninja-build ; \
    ninja-build install; \
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
    ninja-build; \
    ninja-build install; \
    rm -rf /laz-perf


RUN mkdir /nitro; cd /nitro; \
    git clone https://github.com/hobu/nitro; \
    cd nitro; \
    mkdir build; \
    cd build; \
    cmake ..\
        -G Ninja -DCMAKE_INSTALL_PREFIX=/usr/ \
    ; \
    ninja-build; \
    ninja-build install \
    && rm -rf /nitro

ARG PDAL_VERSION=hobu-catchup



RUN pip3 install  numpy

RUN git clone  --branch ${PDAL_VERSION}  https://github.com/PDAL/PDAL.git pdal-git; \
    cd pdal-git;  \
    mkdir build;\
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_INSTALL_PREFIX=/usr/ \
        -DCMAKE_PREFIX_PATH:FILEPATH="${DESTDIR}" \
        -DCMAKE_SYSTEM_PREFIX_PATH="${DESTDIR}/usr" \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_I3S=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DWITH_LAZPERF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_ZLIB=ON \
        -DWITH_TESTS=ON \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    ninja-build; \
    ninja-build install;

RUN  rm /build/usr/lib/*.a

FROM centos:7.6.1810 as runner

RUN yum update -y &&\
    yum install epel-release -y && \
    yum install centos-release-scl -y && \
    yum install -y  \
        zlib \
        python34 \
        libxml2 \
        curl \
        sqlite \
        libtiff \
        geos



COPY --from=builder  /build/usr/bin/ /usr/bin/
COPY --from=builder  /build/usr/lib/ /usr/lib/
COPY --from=builder  /build/usr/include/ /usr/include/
COPY --from=builder  /build/usr/share/ /usr/share/

RUN ldconfig
