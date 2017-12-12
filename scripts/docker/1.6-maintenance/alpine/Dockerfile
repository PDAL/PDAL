FROM alpine:3.6

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
        jsoncpp-dev \
        py-packaging \
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
        jsoncpp \
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
    wget https://github.com/PDAL/PDAL/archive/1.6-maintenance.tar.gz; \
    tar -xf 1.6-maintenance.tar.gz; \
    rm 1.6-maintenance.tar.gz; \
    cd /PDAL-1.6-maintenance; \
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
    rm -rf /PDAL-1.6-maintenance; \
    apk del .build-deps

CMD ["pdal"]
