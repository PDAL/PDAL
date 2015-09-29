#!/bin/bash -e
NUMTHREADS=2
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS

# Installs GeoWave library
mkdir geowave
cd geowave
wget http://s3.amazonaws.com/geowave-rpms/release/TARBALL/geowave-0.8.8.1-24f1a85-jace-source.tar.gz
tar -xzf geowave-*-jace-source.tar.gz
mkdir build
cd build
cmake -G "Unix Makefiles" \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      ..

make -j $NUMTHREADS

# Configure library paths
chmod 777 /home/vagrant/geowave/build/libjace.so
sudo ln -s /home/vagrant/geowave/build/libjace.so /usr/lib/libjace.so
echo "/usr/lib/jvm/java-7-oracle/jre/lib/amd64/server" | sudo tee --append /etc/ld.so.conf.d/jvm.conf
sudo ldconfig

# Install GeoWave as a service and configure to run at startup
# Note: tr removes carriage returns and copies the file
sudo tr -d '\r' < /vagrant/scripts/linux-install-scripts/geowave > /etc/init.d/geowave
sudo chmod 755 /etc/init.d/geowave
sudo update-rc.d geowave defaults
sudo service geowave start
