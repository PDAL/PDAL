FROM alpine:3.7

RUN \
    echo "http://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories; \
    apk update; \
    apk add --no-cache --virtual .build-deps \
        alpine-sdk \
        unzip \
        cmake \
        eigen-dev \
        hexer-dev \
        nitro-dev \
        gdal-dev \
        geos-dev \
        laz-perf-dev \
        libgeotiff-dev \
        libxml2-dev \
        python-dev \
        py-numpy-dev \
        jsoncpp-dev \
        hdf5-dev \
        proj4-dev \
        cpd-dev \
        fgt-dev \
        sqlite-dev \
        postgresql-dev \
        curl-dev \
        linux-headers \
        laszip-dev \
        libspatialite-dev \
        cython-dev \
        py-packaging \
        clang \
        libharu-dev \
        poppler-dev \
        boost-dev \
        zlib-dev \
        laszip-dev \
    ; \
    apk add --no-cache \
        hexer \
        nitro \
        gdal \
        geos \
        laz-perf \
        libgeotiff \
        libxml2 \
        python \
        py-numpy \
        hdf5 \
        proj4 \
        cpd \
        fgt \
        sqlite \
        postgresql \
        libcurl \
        laszip \
        libspatialite \
        jsoncpp \
        py-setuptools \
        libharu \
        poppler \
        boost \
        laszip \
    ;\
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
    git clone https://github.com/hobu/jsoncpp.git /jsoncpp; \
    cd /jsoncpp; \
    export CXX=/usr/bin/clang++; \
    export CC=/usr/bin/clang; \
    cmake -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_STATIC_LIBS=OFF \
        -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    ; \
    make -j2; \
    make install; \
    cd /; \
    rm -rf /jsoncpp; \
    git clone https://github.com/LASzip/LASzip.git /laszip; \
    cd /laszip; \
    cmake -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DBUILD_SHARED_LIBS=ON \
        -DBUILD_STATIC_LIBS=OFF \
    ; \
    make -j2; \
    make install; \
    cd /; \
    rm -rf /laszip; \
    git clone https://github.com/PDAL/PDAL.git /pdal; \
    cd /pdal; \
    git checkout master; \
    mkdir -p _build; \
    cd _build; \
    cmake .. \
        -G "Unix Makefiles" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_C_COMPILER=gcc \
        -DCMAKE_CXX_COMPILER=g++ \
        -DCMAKE_MAKE_PROGRAM=make \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_CPD=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DWITH_LASZIP=ON \
        -DWITH_LAZPERF=ON \
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
    rm -rf /PRC; \
    apk del .build-deps

#CMD ["pdal"]

