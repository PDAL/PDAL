FROM ubuntu:16.04
MAINTAINER Howard Butler <howard@hobu.co>

ENV CC gcc
ENV CXX g++

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
        libsqlite3-mod-spatialite \
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
        clang-3.6 \
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
        cython \
        python-pip \
        libgdal1-dev \
        gdal-bin \
        libpcl-dev \
        time \
        libhpdf-dev \
        python-setuptools \
        libgeos++-dev \
        libhpdf-dev \
        unzip \
        libopenscenegraph-dev \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.6 20 && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.6 20

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

#RUN git clone --depth=1 https://github.com/OSGeo/gdal.git \
#    &&    cd gdal/gdal \
#    && ./configure --prefix=/usr \
#            --mandir=/usr/share/man \
#            --includedir=/usr/include/gdal \
#            --with-threads \
#            --with-grass=no \
#            --with-hide-internal-symbols=yes \
#            --with-rename-internal-libtiff-symbols=yes \
#            --with-rename-internal-libgeotiff-symbols=yes \
#            --with-libtiff=internal \
#            --with-geotiff=internal \
#            --with-webp \
#            --with-jasper \
#            --with-netcdf \
#            --with-hdf5=/usr/lib/x86_64-linux-gnu/hdf5/serial/ \
#            --with-xerces \
#            --with-geos \
#            --with-sqlite3 \
#            --with-curl \
#            --with-pg \
#            --with-mysql \
#            --with-python \
#            --with-odbc \
#            --with-ogdi \
#            --with-dods-root=/usr \
#            --with-spatialite=/usr \
#            --with-cfitsio=no \
#            --with-ecw=no \
#            --with-mrsid=no \
#            --with-poppler=yes \
#            --with-openjpeg=yes \
#            --with-freexl=yes \
#            --with-libkml=yes \
#            --with-armadillo=yes \
#            --with-liblzma=yes \
#            --with-epsilon=/usr \
#    && make -j 4 \
#    && make install \
#    && rm -rf /gdal

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
    && git checkout e7065cbc5bdbbe0c6e50c9d93d1cd346e9be6778 \
    && mkdir build \
    && cd build \
    && cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
        .. \
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

RUN wget http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz \
        && tar -xvf 3.2.7.tar.gz \
        && cp -R eigen-eigen-b30b87236a1b/Eigen/ /usr/include/Eigen/ \
        && cp -R eigen-eigen-b30b87236a1b/unsupported/ /usr/include/unsupported/ \
        && rm -rf /3.2.7.tar.gz \
        && rm -rf /eigen-eigen-b30b87236a1b

#RUN git clone https://github.com/PointCloudLibrary/pcl.git \
#        && cd pcl \
#        && git checkout pcl-1.8.0 \
#        && mkdir build \
#        && cd build \
#        && CC="clang" CXX="clang++" CXXFLAGS="-std=c++11"  cmake \
#                -DBUILD_2d=ON \
#                -DBUILD_CUDA=OFF \
#                -DBUILD_GPU=OFF \
#                -DBUILD_apps=OFF \
#                -DBUILD_common=ON \
#                -DBUILD_examples=OFF \
#                -DBUILD_features=ON \
#                -DBUILD_filters=ON \
#                -DBUILD_geometry=ON \
#                -DBUILD_global_tests=OFF \
#                -DBUILD_io=ON \
#                -DBUILD_kdtree=ON \
#                -DBUILD_keypoints=ON \
#                -DBUILD_ml=ON \
#                -DBUILD_octree=ON \
#                -DBUILD_outofcore=OFF \
#                -DBUILD_people=OFF \
#                -DBUILD_recognition=OFF \
#                -DBUILD_registration=ON \
#                -DBUILD_sample_concensus=ON \
#                -DBUILD_search=ON \
#                -DBUILD_segmentation=ON \
#                -DBUILD_simulation=OFF \
#                -DBUILD_stereo=OFF \
#                -DBUILD_surface=ON \
#                -DBUILD_surface_on_nurbs=OFF \
#                -DBUILD_tools=OFF \
#                -DBUILD_tracking=OFF \
#                -DBUILD_visualization=OFF \
#                -DWITH_LIBUSB=OFF \
#                -DWITH_OPENNI=OFF \
#                -DWITH_OPENNI2=OFF \
#                -DWITH_FZAPI=OFF \
#                -DWITH_PXCAPI=OFF \
#                -DWITH_PNG=OFF \
#                -DWITH_QHULL=OFF \
#                -DWITH_QT=OFF \
#                -DWITH_VTK=OFF \
#                -DWITH_PCAP=OFF \
#                -DWITH_OPENNI=OFF \
#                -DWITH_OPENNI2=OFF \
#                -DWITH_FZAPI=OFF \
#                -DWITH_ENSENSO=OFF \
#                -DWITH_DAVIDSDK=OFF \
#                -DWITH_DSSDK=OFF \
#                -DWITH_RSSDK=OFF \
#                -DWITH_OPENGL=OFF \
#                -DCMAKE_INSTALL_PREFIX=/usr \
#                -DCMAKE_BUILD_TYPE="Release" \
#                .. \
#        && make -j 4\
#        && make install \
#        && rm -rf /pcl



RUN svn co -r 2691 https://svn.osgeo.org/metacrs/geotiff/trunk/libgeotiff/ \
    && cd libgeotiff \
    && ./autogen.sh \
    && ./configure --prefix=/usr \
    && make \
    && make install \
    && rm -rf /libgeotiff



RUN git clone --depth 1 --branch v0.4.6 https://github.com/gadomski/fgt.git \
    && cd fgt \
    && cmake . \
        -DWITH_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/include \
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
        -DEIGEN3_INCLUDE_DIR=/usr/include \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install \
    && rm -rf /cpd

RUN git clone https://github.com/ninja-build/ninja.git \
    && cd ninja \
    && ./configure.py --bootstrap \
    && cp ninja /usr/bin

# Install Java.
RUN echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | debconf-set-selections && \
    add-apt-repository -y ppa:webupd8team/java && \
    apt-get update && \
    apt-get install -y oracle-java8-installer && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf /var/cache/oracle-jdk8-installer

ENV JAVA_HOME /usr/lib/jvm/java-8-oracle

# fixup for PCL 1.7
RUN ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so

RUN add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y \
    && apt-get update \
    && apt-get install -y mbsystem mbsystem-dev \
    && rm -rf /var/lib/apt/lists/*



#RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.6 20 && update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.6 20


