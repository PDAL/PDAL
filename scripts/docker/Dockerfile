FROM pdal/dependencies:latest
MAINTAINER Howard Butler <howard@hobu.co>
ARG branch=master

ENV CC clang
ENV CXX clang++

RUN git clone https://github.com/PDAL/PDAL \
	&& cd PDAL \
	&& git checkout ${branch} \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DBUILD_PLUGIN_ATTRIBUTE=ON \
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
		-DWITH_GEOTIFF=ON \
		-DWITH_LASZIP=ON \
		-DWITH_TESTS=ON \
		.. \
	&& make -j4 \
	&& make install \
	&& cd / \
	&& rm -rf PDAL


# cleanup
RUN rm -rf laszip laz-perf points2grid pcl nitro hexer 3.2.7.tar.gz eigen-eigen-b30b87236a1b

ENTRYPOINT ["/usr/bin/pdal"]
