FROM ubuntu:15.04
MAINTAINER Howard Butler <howard@hobu.co>

RUN apt-key adv --recv-keys --keyserver hkp://keyserver.ubuntu.com:80 16126D3A3E5C1192
RUN apt-get update -qq
RUN apt-get -qq remove postgis

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
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
        libboost-filesystem1.55-dev \
        libboost-iostreams1.55-dev \
        libboost-program-options1.55-dev \
        libboost-system1.55-dev \
        libboost-thread1.55-dev \
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
        python-pip \
    && rm -rf /var/lib/apt/lists/*


RUN git clone --depth=1 https://github.com/OSGeo/gdal.git \
    &&    cd gdal/gdal \
    && ./configure --prefix=/usr \
            --mandir=/usr/share/man \
            --includedir=/usr/include/gdal \
            --with-threads \
            --with-grass=no \
            --with-hide-internal-symbols=yes \
            --with-rename-internal-libtiff-symbols=yes \
            --with-rename-internal-libgeotiff-symbols=yes \
            --with-libtiff=internal \
            --with-geotiff=internal \
            --with-webp \
            --with-jasper \
            --with-netcdf \
            --with-hdf5=/usr/lib/x86_64-linux-gnu/hdf5/serial/ \
            --with-xerces \
            --with-geos \
            --with-sqlite3 \
            --with-curl \
            --with-pg \
            --with-mysql \
            --with-python \
            --with-odbc \
            --with-ogdi \
            --with-dods-root=/usr \
            --with-spatialite=/usr \
            --with-cfitsio=no \
            --with-ecw=no \
            --with-mrsid=no \
            --with-poppler=yes \
            --with-openjpeg=yes \
            --with-freexl=yes \
            --with-libkml=yes \
            --with-armadillo=yes \
            --with-liblzma=yes \
            --with-epsilon=/usr \
    && make -j 4 \
    && make install \
    && rm -rf gdal

RUN git clone https://github.com/hobu/nitro \
    && cd nitro \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        .. \
    && make \
    && make install \
    && rm -rf nitro

RUN git clone https://github.com/LASzip/LASzip.git laszip \
    && cd laszip \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
    && make \
    && make install \
    && rm -rf laszip


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
    && rm -rf hexer

RUN git clone https://github.com/CRREL/points2grid.git \
    && cd points2grid \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
    && make \
    && make install \
    && rm -rf points2grid

RUN git clone  https://github.com/verma/laz-perf.git \
    && cd laz-perf \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
    && make \
    && make install \
    && rm -rf laz-perf

RUN wget http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz \
        && tar -xvf 3.2.7.tar.gz \
        && cp -R eigen-eigen-b30b87236a1b/Eigen/ /usr/include/Eigen/ \
        && cp -R eigen-eigen-b30b87236a1b/unsupported/ /usr/include/unsupported/

RUN git clone https://github.com/chambbj/pcl.git \
        && cd pcl \
        && git checkout pcl-1.7.2-sans-opengl \
        && mkdir build \
        && cd build \
        && cmake \
                -DBUILD_2d=ON \
                -DBUILD_CUDA=OFF \
                -DBUILD_GPU=OFF \
                -DBUILD_apps=OFF \
                -DBUILD_common=ON \
                -DBUILD_examples=OFF \
                -DBUILD_features=ON \
                -DBUILD_filters=ON \
                -DBUILD_geometry=ON \
                -DBUILD_global_tests=OFF \
                -DBUILD_io=ON \
                -DBUILD_kdtree=ON \
                -DBUILD_keypoints=ON \
                -DBUILD_ml=ON \
                -DBUILD_octree=ON \
                -DBUILD_outofcore=OFF \
                -DBUILD_people=OFF \
                -DBUILD_recognition=OFF \
                -DBUILD_registration=ON \
                -DBUILD_sample_concensus=ON \
                -DBUILD_search=ON \
                -DBUILD_segmentation=ON \
                -DBUILD_simulation=OFF \
                -DBUILD_stereo=OFF \
                -DBUILD_surface=ON \
                -DBUILD_surface_on_nurbs=OFF \
                -DBUILD_tools=OFF \
                -DBUILD_tracking=OFF \
                -DBUILD_visualization=OFF \
                -DWITH_LIBUSB=OFF \
                -DWITH_OPENNI=OFF \
                -DWITH_OPENNI2=OFF \
                -DWITH_FZAPI=OFF \
                -DWITH_PXCAPI=OFF \
                -DWITH_PNG=OFF \
                -DWITH_QHULL=OFF \
                -DWITH_QT=OFF \
                -DWITH_VTK=OFF \
                -DWITH_PCAP=OFF \
                -DCMAKE_INSTALL_PREFIX=/usr \
                -DCMAKE_BUILD_TYPE="Release" \
                .. \
        && make \
        && make install \
        && rm -rf pcl



RUN svn co -r 2691 https://svn.osgeo.org/metacrs/geotiff/trunk/libgeotiff/ \
    && cd libgeotiff \
    && ./autogen.sh \
    && ./configure --prefix=/usr \
    && make \
    && make install \
    && rm -rf libgeotiff

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
        ninja-build \
        libgeos++-dev \
        unzip \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /vdatum \
    && cd /vdatum \
    && wget http://download.osgeo.org/proj/vdatum/usa_geoid2012.zip && unzip -j -u usa_geoid2012.zip -d /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/usa_geoid2009.zip && unzip -j -u usa_geoid2009.zip -d /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/usa_geoid2003.zip && unzip -j -u usa_geoid2003.zip -d /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/usa_geoid1999.zip && unzip -j -u usa_geoid1999.zip -d /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/vertcon/vertconc.gtx && mv vertconc.gtx /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/vertcon/vertcone.gtx && mv vertcone.gtx /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/vertcon/vertconw.gtx && mv vertconw.gtx /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/egm96_15/egm96_15.gtx && mv egm96_15.gtx /usr/share/proj \
    && wget http://download.osgeo.org/proj/vdatum/egm08_25/egm08_25.gtx && mv egm08_25.gtx /usr/share/proj \
    && rm -rf /vdatum

RUN rm -rf laszip laz-perf points2grid pcl nitro hexer 3.2.7.tar.gz eigen-eigen-b30b87236a1b gdal libgeotiff
RUN apt-get clean


RUN git clone https://github.com/gadomski/armadillo.git \
    && cd armadillo/branch-5.600 \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf armadillo

ADD fgt-pr-13.patch /

RUN git clone https://github.com/gadomski/fgt.git \
    && cd fgt \
    && git checkout b3bcea4767fc0fd018bd280a1bc8e8d6c44c513e \
    && patch -p1 < /fgt-pr-13.patch \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf fgt

RUN git clone https://github.com/gadomski/cpd.git \
    && cd cpd \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DFgt_DIR=/usr \
    && make \
    && make install \
    && mv /usr/lib/cpd/cmake/CpdConfig.cmake /usr/lib/cpd/cmake/CPDConfig.cmake \
    && mv /usr/lib/cpd/cmake/CpdConfigVersion.cmake /usr/lib/cpd/cmake/CPDConfigVersion.cmake \
    && rm -rf cpd


