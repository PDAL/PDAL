sudo apt-get update -qq
sudo apt-get install -y -q build-essential
sudo apt-get install -y python-software-properties software-properties-common python g++ make cmake wget git
sudo add-apt-repository -y ppa:ubuntugis/ubuntugis-unstable
sudo add-apt-repository -y ppa:webupd8team/java
sudo apt-get update -qq
echo debconf shared/accepted-oracle-license-v1-1 select true | sudo debconf-set-selections
echo debconf shared/accepted-oracle-license-v1-1 seen true | sudo debconf-set-selections
sudo apt-get install -y -q  git \
                           cmake \
                           libgeos-dev \
                           libgdal-dev \
                           libpq-dev \
                           python-all-dev \
                           python-numpy \
                           libproj-dev \
                           libtiff4-dev \
                           libxml2-dev \
                           libboost-all-dev \
                           libbz2-dev \
                           libsqlite0-dev \
                           cmake-curses-gui \
                           screen \
                           postgis \
                           libcunit1-dev \
                           postgresql-server-dev-9.3 \
                           postgresql-9.3-postgis-2.1 \
                           libmsgpack-dev \
                           libgeos++-dev \
                           vim \
                           libeigen3-dev \
                           libflann-dev \
                           libglew-dev \
                           libhdf5-serial-dev \
                           libjsoncpp-dev \
                           vtk6 \
                           libvtk6-dev \
                           gcc-multilib \
                           g++-multilib \
                           libglew-dev \
                           oracle-java7-installer \
                           maven \
                           libc6-i386



