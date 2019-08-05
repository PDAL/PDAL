FROM pdal/dependencies
MAINTAINER Pete Gadomski <pete.gadomski@gmail.com>

COPY rivlib-2_3_0-x86_64-linux-gcc55.zip /
RUN unzip rivlib-2_3_0-x86_64-linux-gcc55.zip

RUN CC="gcc" CXX="g++" git clone --depth=1 https://github.com/PDAL/PDAL \
	&& cd PDAL \
	&& git checkout master \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DBUILD_PLUGIN_CPD=ON \
		-DBUILD_PLUGIN_GREYHOUND=ON \
		-DBUILD_PLUGIN_ICEBRIDGE=ON \
		-DBUILD_PLUGIN_MRSID=ON \
		-DBUILD_PLUGIN_NITF=ON \
		-DBUILD_PLUGIN_OCI=OFF \
		-DBUILD_PLUGIN_P2G=ON \
		-DBUILD_PLUGIN_PGPOINTCLOUD=ON \
		-DBUILD_PLUGIN_SQLITE=ON \
        -DRiVLib_DIR=/rivlib-2_3_0-x86_64-linux-gcc55/cmake \
		-DBUILD_PLUGIN_RIVLIB=ON \
		-DBUILD_PLUGIN_PYTHON=ON \
		-DCMAKE_INSTALL_PREFIX=/usr \
		-DENABLE_CTEST=OFF \
		-DWITH_APPS=ON \
		-DWITH_LAZPERF=ON \
		-DWITH_LASZIP=ON \
		-DWITH_TESTS=ON \
		-DCMAKE_BUILD_TYPE=Release \
		.. \
	&& make -j4 \
	&& make install \
    && rm -rf PDAL
