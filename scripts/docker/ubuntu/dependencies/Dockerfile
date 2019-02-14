FROM ubuntu:18.04
MAINTAINER Howard Butler <howard@hobu.co>

ENV CC gcc
ENV CXX g++

RUN apt-get update -qq && apt-get install gnupg2 -y
RUN apt-key adv --recv-keys --keyserver hkp://keyserver.ubuntu.com:80 16126D3A3E5C1192

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
        libgeotiff-epsg \
        openssh-client \
        python-dev \
        python-numpy \
        software-properties-common \
        wget \
        automake \
        libtool \
        libspatialite-dev \
        libsqlite3-mod-spatialite \
        subversion \
        libjsoncpp-dev \
        libboost-filesystem1.65-dev \
        libboost-iostreams1.65-dev \
        libboost-program-options1.65-dev \
        libboost-system1.65-dev \
        libboost-thread1.65-dev \
        subversion \
        clang \
        clang-6.0 \
        libproj-dev \
        libc6-dev \
        libnetcdf-dev \
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
        libspatialite-dev \
        libdap-dev\
        cython \
        python-pip \
        libpcl-dev \
        time \
        libhpdf-dev \
        python-setuptools \
        libhpdf-dev \
        unzip \
        libopenscenegraph-dev \
        libzstd-dev \
        liblzma-dev \
        libgdal-dev \
        libeigen3-dev \
        proj-data \
        ninja-build \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/hobu/nitro \
    && cd nitro \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        .. \
    && make \
    && make install \
    && rm -rf /nitro

RUN git clone https://github.com/LASzip/LASzip.git laszip \
    && cd laszip \
    && git checkout 3.1.1 \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        . \
    && make \
    && make install \
    && rm -rf /laszip


RUN git clone https://github.com/hobu/hexer.git \
    && cd hexer \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
    && make \
    && make install \
    && rm -rf /hexer

RUN git clone  https://github.com/hobu/laz-perf.git \
    && cd laz-perf \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
    && make \
    && make install \
    && rm -rf /laz-perf


RUN git clone --depth 1 --branch v0.4.6 https://github.com/gadomski/fgt.git \
    && cd fgt \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf /fgt

RUN git clone --depth 1 --branch v0.5.1 https://github.com/gadomski/cpd.git \
    && cd cpd \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DWITH_JSONCPP=OFF \
        -DWITH_FGT=ON \
        -DWITH_STRICT_WARNINGS=OFF \
        -DWITH_DOCS=OFF \
        -DEIGEN3_INCLUDE_DIR=/usr/include/eigen3 \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf /cpd


# Install Java.
RUN echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | debconf-set-selections && \
    add-apt-repository -y ppa:webupd8team/java && \
    apt-get update && \
    apt-get install -y oracle-java8-installer && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /var/cache/oracle-jdk8-installer

ENV JAVA_HOME /usr/lib/jvm/java-8-oracle

# fixup for PCL 1.7
RUN ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so /usr/lib/libvtkproj4.so



