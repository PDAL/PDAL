FROM ubuntu:14.04
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
		libtiff4-dev \
		openssh-client \
		python-dev \
		python-numpy \
		python-numpy \
		python-software-properties \
		software-properties-common \
		wget \
	&& rm -rf /var/lib/apt/lists/*

RUN add-apt-repository ppa:ubuntugis/ubuntugis-unstable -y
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
		# from ppa:ubuntu-toolchain-r/test
		g++-4.8 \
		clang \
		ninja-build \
		# from ppa:boost-latest/ppa
		libboost-filesystem1.55-dev \
		libboost-iostreams1.55-dev \
		libboost-program-options1.55-dev \
		libboost-system1.55-dev \
		libboost-thread1.55-dev \
#
		# from ppa:ubuntugis/ubuntugis-unstable
		libgdal1h \
		libgdal-dev \
		gdal-bin \
		libgeos++-dev \
		libproj-dev \

		# from ppa:ubuntugis/ppa
		libgeotiff-dev \
		libxml2-dev \
#
	&& rm -rf /var/lib/apt/lists/*

# Get howard@hobu.co key
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys BFE1B014

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 50


RUN git clone https://github.com/hobu/nitro \
	&& cd nitro \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DCMAKE_INSTALL_PREFIX=/usr \
		.. \
	&& make \
	&& make install

RUN git clone https://github.com/LASzip/LASzip.git laszip \
	&& cd laszip \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DCMAKE_INSTALL_PREFIX=/usr \
		-DCMAKE_BUILD_TYPE="Release" \
		.. \
	&& make \
	&& make install


RUN git clone https://github.com/hobu/hexer.git \
	&& cd hexer \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DCMAKE_INSTALL_PREFIX=/usr \
		-DCMAKE_BUILD_TYPE="Release" \
		.. \
	&& make \
	&& make install

RUN git clone https://github.com/CRREL/points2grid.git \
	&& cd points2grid \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DCMAKE_INSTALL_PREFIX=/usr \
		-DCMAKE_BUILD_TYPE="Release" \
		.. \
	&& make \
	&& make install

RUN git clone  https://github.com/verma/laz-perf.git \
	&& cd laz-perf \
	&& mkdir build \
	&& cd build \
	&& cmake \
		-DCMAKE_INSTALL_PREFIX=/usr \
		-DCMAKE_BUILD_TYPE="Release" \
		.. \
	&& make \
	&& make install

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
        && make install

RUN apt-get update && apt-get install -y --fix-missing --no-install-recommends \
		subversion \
		libjsoncpp-dev

RUN svn co -r 2691 https://svn.osgeo.org/metacrs/geotiff/trunk/libgeotiff/ \
    && cd libgeotiff \
    &&  cmake . -DCMAKE_INSTALL_PREFIX=/usr -DWITH_TIFF=ON -DWITH_PROJ4=ON \
    && make \
    && make install
#RUN wget https://cmake.org/files/v3.4/cmake-3.4.1.tar.gz \
#    && tar zxvf cmake-3.4.1.tar.gz \
#    && cd cmake-3.4.1 \
#    && ./bootstrap --prefix=/usr \
#    && make \
#    && make install

RUN wget http://sourceforge.net/projects/arma/files/armadillo-6.400.2.tar.gz \
    && tar zxvf armadillo-6.400.2.tar.gz \
    && cd armadillo-6.400.2 \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install

RUN git clone https://github.com/gadomski/fgt.git \
    && cd fgt \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release \
    && make \
    && make install

RUN git clone https://github.com/hobu/cpd.git \
    && cd cpd \
    && git checkout armadillo-6 \
    && cmake . -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release -DFgt_DIR=/usr \
    && make \
    && make install

RUN rm -rf /cpd /armadillo* /fgt /libgeotiff /pcl /eigen* /points2grid /nitro /*.gz /laszip /laz-perf /hexer
