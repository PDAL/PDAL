FROM pdal/dependencies:latest
MAINTAINER Howard Butler <howard@hobu.co>

ENV CC gcc
ENV CXX g++

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
        libhpdf-dev \
    libsqlite3-mod-spatialite \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth=1 https://github.com/PDAL/PDAL \
    && cd PDAL \
    && git checkout master \
    && mkdir build \
    && cd build \
    && cmake \
        -DBUILD_PLUGIN_CPD=OFF \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_MRSID=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_OCI=OFF \
        -DBUILD_PLUGIN_P2G=ON \
        -DBUILD_PLUGIN_PCL=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_RIVLIB=OFF \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DENABLE_CTEST=OFF \
        -DWITH_APPS=ON \
        -DWITH_LAZPERF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_TESTS=ON \
        -DCMAKE_BUILD_TYPE=Release \
        .. \
    && make -j 2\
    && make install \
    && rm -rf /PDAL


RUN pip install packaging \
    && pip install PDAL

RUN git clone https://github.com/PDAL/PRC.git \
    && cd PRC \
    && git checkout master \
    && mkdir build \
    && cd build \
    && echo `pwd` \
    && ls .. \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DPDAL_DIR=/usr/lib/pdal/cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        .. \
    && make \
    && make install \
    && rm -rf /PRC

# Remove all our dev junk and then we'll put back only
# runtime stuff
RUN apt-get purge -y \
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
    liblapack-dev \
    libtiff5-dev \
    openssh-client \
    python-dev \
    python-numpy \
    python-software-properties \
    software-properties-common \
    wget \
    automake \
    libtool \
    libspatialite-dev \
    libhdf5-dev \
    subversion \
    libjsoncpp-dev \
    libboost-filesystem1.58-dev \
    libboost-iostreams1.58-dev \
    libboost-program-options1.58-dev \
    libboost-system1.58-dev \
    libboost-thread1.58-dev \
    subversion \
    clang \
    libproj-dev \
    libc6-dev \
    libnetcdf-dev \
    libjasper-dev \
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
    libgeos-dev \
    libmysqlclient-dev \
    libltdl-dev \
    libcurl4-openssl-dev \
    libspatialite-dev \
    libdap-dev\
    ninja \
    cython \
    python-pip

RUN apt-get autoremove -y

RUN apt-get update && apt-get install -y \
    libexpat1 \
    libgomp1 \
    libxml2 \
    libgeos-c1v5 \
    libjsoncpp1 \
    libcurl3 \
    libarmadillo6 \
    libwebp5 \
    libodbc1 \
    odbcinst1debian2 \
    libxerces-c3.1 \
    libjasper1 \
    netcdf-bin \
    libhdf4-0-alt \
    libgif7 \
    libpq5 \
    libdapclient6v5 \
    libspatialite7 \
    libsqlite3-mod-spatialite \
    spatialite-bin \
    libmysqlclient20 \
    libtiff5 \
    libboost-system1.58.0 \
    libboost-filesystem1.58.0 \
    libboost-thread1.58.0 \
    libboost-program-options1.58.0 \
    libboost-iostreams1.58.0 \
    libboost-date-time1.58.0 \
    libboost-serialization1.58.0 \
    libboost-chrono1.58.0 \
    libboost-atomic1.58.0 \
    libboost-regex1.58.0 \
    libgdal1i \
    libflann1.8 \
    libpython2.7 \
    libhdf5-cpp-11
