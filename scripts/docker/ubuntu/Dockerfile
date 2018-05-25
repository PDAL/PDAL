FROM pdal/dependencies:ubuntu
MAINTAINER Howard Butler <howard@hobu.co>

RUN     git clone https://github.com/PDAL/PDAL.git /pdal; \
    cd /pdal; \
    git checkout 1.7-maintenance; \
    mkdir -p _build; \
    cd _build; \
    cmake .. \
        -G "Unix Makefiles" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_MAKE_PROGRAM=make \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_CPD=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_PCL=ON \
        -DWITH_LASZIP=ON \
        -DWITH_LAZPERF=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_INSTALL_LIBDIR=lib \
    ; \
    make -j2; \
    make install; \
    cd ../python; \
    python setup.py install; \
    cd /; \
    rm -rf /pdal; \
    git clone https://github.com/PDAL/PRC.git ; \
    cd PRC; \
    git checkout master ; \
    mkdir build ; \
    cd build ; \
    cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DPDAL_DIR=/usr/lib/pdal/cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        .. ; \
    make; \
    make install; \
    cd /; \
    rm -rf /PRC;

RUN    apt-get purge -y \
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
        libboost-filesystem1.65-dev \
        libboost-iostreams1.65-dev \
        libboost-program-options1.65-dev \
        libboost-system1.65-dev \
        libboost-thread1.65-dev \
        subversion \
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
        cython \
        python-pip \
    ; \
    apt-get autoremove -y; \
    apt-get update; \
    apt-get install -y \
        libexpat1 \
        libgomp1 \
        libxml2 \
        libgeos-c1v5 \
        libjsoncpp1 \
        libcurl3 \
        libarmadillo8 \
        libwebp6 \
        libodbc1 \
        odbcinst1debian2 \
        libxerces-c3.2 \
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
        libboost-system1.65.1 \
        libboost-filesystem1.65.1 \
        libboost-thread1.65.1 \
        libboost-program-options1.65.1 \
        libboost-iostreams1.65.1 \
        libboost-date-time1.65.1 \
        libboost-serialization1.65.1 \
        libboost-chrono1.65.1 \
        libboost-atomic1.65.1 \
        libboost-regex1.65.1 \
        libflann1.9 \
        libpython2.7 \
        libhdf5-cpp-100 \
        libpcl-common1.8 \
        libpcl-features1.8 \
        libpcl-filters1.8 \
        libpcl-io1.8 \
        libpcl-kdtree1.8 \
        libpcl-keypoints1.8 \
        libpcl-octree1.8 \
        libpcl-outofcore1.8 \
        libpcl-people1.8 \
        libpcl-recognition1.8 \
        libpcl-registration1.8 \
        libpcl-sample-consensus1.8 \
        libpcl-search1.8 \
        libpcl-segmentation1.8 \
        libpcl-surface1.8 \
        libpcl-tracking1.8 \
        libpcl-visualization1.8


