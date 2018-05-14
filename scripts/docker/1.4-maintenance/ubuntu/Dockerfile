FROM ubuntu:16.04
MAINTAINER Howard Butler <howard@hobu.co>

ENV CC gcc
ENV CXX g++
ENV JAVA_HOME /usr/lib/jvm/java-8-oracle

RUN \
    apt-key adv --recv-keys --keyserver hkp://keyserver.ubuntu.com:80 16126D3A3E5C1192; \
    apt-get update -qq; \
    apt-get -qq remove postgis; \
    echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | debconf-set-selections; \
    apt-get install -y --fix-missing --no-install-recommends \
        software-properties-common \
    ; \
    add-apt-repository -y ppa:webupd8team/java; \
    add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y; \
    apt-get update; \
    apt-get install -y --fix-missing --no-install-recommends \
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
        mbsystem \
        mbsystem-dev \
        oracle-java8-installer \
    ; \
    rm -rf /var/lib/apt/lists/*; \
    rm -rf /var/cache/oracle-jdk8-installer; \
    update-alternatives --install /usr/bin/clang clang /usr/bin/clang-3.6 20; \
    update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-3.6 20; \
    ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so; \
    git clone https://github.com/hobu/nitro; \
    cd nitro; \
    mkdir build; \
    cd build; \
    cmake ..\
        -DCMAKE_INSTALL_PREFIX=/usr \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /nitro; \
    git clone https://github.com/LASzip/LASzip.git laszip; \
    cd laszip; \
    git checkout e7065cbc5bdbbe0c6e50c9d93d1cd346e9be6778; \
    mkdir build; \
    cd build; \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /laszip; \
    git clone https://github.com/hobu/hexer.git; \
    cd hexer; \
    mkdir build; \
    cd build; \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /hexer; \
    git clone  https://github.com/hobu/laz-perf.git; \
    cd laz-perf; \
    mkdir build; \
    cd build; \
    cmake .. \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE="Release" \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /laz-perf; \
    wget http://bitbucket.org/eigen/eigen/get/3.2.7.tar.gz; \
    tar -xvf 3.2.7.tar.gz; \
    cp -R eigen-eigen-b30b87236a1b/Eigen/ /usr/include/Eigen/; \
    cp -R eigen-eigen-b30b87236a1b/unsupported/ /usr/include/unsupported/; \
    rm -rf /3.2.7.tar.gz; \
    rm -rf /eigen-eigen-b30b87236a1b; \
    svn co -r 2691 https://svn.osgeo.org/metacrs/geotiff/trunk/libgeotiff/; \
    cd libgeotiff; \
    ./autogen.sh; \
    ./configure --prefix=/usr; \
    make; \
    make install; \
    cd /; \
    rm -rf /libgeotiff; \
    git clone --depth 1 --branch v0.4.6 https://github.com/gadomski/fgt.git; \
    cd fgt; \
    cmake . \
        -DWITH_TESTS=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DEIGEN3_INCLUDE_DIR=/usr/include \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /fgt; \
    git clone --depth 1 --branch v0.5.0 https://github.com/gadomski/cpd.git; \
    cd cpd; \
    cmake . \
        -DWITH_TESTS=OFF \
        -DWITH_JSONCPP=OFF \
        -DWITH_FGT=ON \
        -DWITH_STRICT_WARNINGS=OFF \
        -DWITH_DOCS=OFF \
        -DEIGEN3_INCLUDE_DIR=/usr/include \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /cpd; \
    mkdir /vdatum; \
    cd /vdatum; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2012.zip && unzip -j -u usa_geoid2012.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2009.zip && unzip -j -u usa_geoid2009.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2003.zip && unzip -j -u usa_geoid2003.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid1999.zip && unzip -j -u usa_geoid1999.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertconc.gtx && mv vertconc.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertcone.gtx && mv vertcone.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertconw.gtx && mv vertconw.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/egm96_15/egm96_15.gtx && mv egm96_15.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/egm08_25/egm08_25.gtx && mv egm08_25.gtx /usr/share/proj; \
    cd /; \
    rm -rf /vdatum; \
    wget https://github.com/PDAL/PDAL/archive/1.4-maintenance.tar.gz; \
    tar -xf 1.4-maintenance.tar.gz; \
    rm 1.4-maintenance.tar.gz; \
    cd /PDAL-1.4-maintenance; \
    mkdir build; \
    cd build; \
    cmake .. \
        -DBUILD_PLUGIN_CPD=OFF \
        -DBUILD_PLUGIN_MBIO=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_MRSID=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_OCI=OFF \
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
        -DWITH_PDAL_JNI=ON \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    make -j2; \
    make install; \
    cd /; \
    rm -rf /PDAL; \
    pip install packaging; \
    pip install PDAL; \
    git clone https://github.com/PDAL/PRC.git; \
    cd PRC; \
    git checkout master; \
    mkdir build; \
    cd build; \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DPDAL_DIR=/usr/lib/pdal/cmake \
        -DCMAKE_INSTALL_PREFIX=/usr \
    ; \
    make; \
    make install; \
    cd /; \
    rm -rf /PRC; \
    apt-get purge -y \
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
        libhdf5-cpp-11 \
        libpcl-common1.7 \
        libpcl-features1.7 \
        libpcl-filters1.7 \
        libpcl-io1.7 \
        libpcl-kdtree1.7 \
        libpcl-keypoints1.7 \
        libpcl-octree1.7 \
        libpcl-outofcore1.7 \
        libpcl-people1.7 \
        libpcl-recognition1.7 \
        libpcl-registration1.7 \
        libpcl-sample-consensus1.7 \
        libpcl-search1.7 \
        libpcl-segmentation1.7 \
        libpcl-surface1.7 \
        libpcl-tracking1.7 \
        libpcl-visualization1.7

