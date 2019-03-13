FROM alpine:3.8

#
# Nitro looks for unistd.h in the wrong place.
#
RUN \
    mkdir -p /usr/include/linux; \
    ln -sf /usr/include/unistd.h /usr/include/linux/unistd.h

#
# To be found at runtime by GDAL, libproj needs to be named plainly.
#
RUN \
    ln -sf /usr/lib/libproj.so.13 /usr/lib/libproj.so

#
# Datum files for proj
#
RUN \
    mkdir -p /usr/share/proj; \
    mkdir /vdatum; \
    cd /vdatum; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2012.zip && unzip -j -o usa_geoid2012.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2009.zip && unzip -j -o usa_geoid2009.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid2003.zip && unzip -j -o usa_geoid2003.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/usa_geoid1999.zip && unzip -j -o usa_geoid1999.zip -d /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertconc.gtx && mv vertconc.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertcone.gtx && mv vertcone.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/vertcon/vertconw.gtx && mv vertconw.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/egm96_15/egm96_15.gtx && mv egm96_15.gtx /usr/share/proj; \
    wget http://download.osgeo.org/proj/vdatum/egm08_25/egm08_25.gtx && mv egm08_25.gtx /usr/share/proj; \
    cd /; \
    rm -rf /vdatum

RUN \
    echo "@edgetesting http://dl-cdn.alpinelinux.org/alpine/edge/testing" >> /etc/apk/repositories; \
    echo "@edgemain http://dl-cdn.alpinelinux.org/alpine/edge/main" >> /etc/apk/repositories; \
    apk update; \
    apk add --no-cache \
        git \
        cmake \
        ninja \
        g++ \
    ; \
    apk add --no-cache \
        eigen-dev \
        py3-numpy \
        py-numpy-dev \
        libxml2-dev \
        python3-dev \
        curl-dev \
        postgresql-dev \
        sqlite-dev \
        libcrypto1.1@edgemain \
        libspatialite-dev@edgetesting \
        gdal-dev@edgetesting \
        libgeotiff-dev@edgetesting \
        nitro@edgetesting \
        nitro-dev@edgetesting \
        laszip-dev@edgetesting \
        laz-perf-dev@edgetesting \
        hdf5-dev@edgetesting \
        proj4@edgetesting \
    ; \
    \
#
# These use PDAL's vendor/eigen
#
    git clone https://github.com/gadomski/fgt.git; \
    cd fgt; \
    mkdir build; \
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    ninja install; \
    cd /; \
    rm -rf fgt; \
    \
    git clone https://github.com/gadomski/cpd.git; \
    cd cpd; \
    mkdir build; \
    cd build; \
    cmake .. \
        -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
    ; \
    ninja install; \
    cd /; \
    rm -rf cpd;
