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
