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
        libspatialite-dev \
        libdap-dev\
        cython \
        python3-pip \
        libpcl-dev \
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
    && rm -rf /var/lib/apt/lists/*
