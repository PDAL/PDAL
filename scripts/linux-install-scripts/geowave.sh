#!/bin/bash -e
# Installs GeoWave library
wget http://s3.amazonaws.com/geowave-rpms/dev/TARBALL/geowave-0.8.7-SNAPSHOT-3785f05-jace-linux-amd64-debug.tar.gz
mkdir geowave
tar -xzf geowave-*-jace-linux-amd64-debug.tar.gz -C /home/vagrant/geowave

# Configure library paths
chmod 777 /home/vagrant/geowave/libjace.so
sudo ln -s /home/vagrant/geowave/libjace.so /usr/lib/libjace.so
echo "/usr/lib/jvm/java-7-oracle/jre/lib/amd64" | sudo tee --append /etc/ld.so.conf.d/awt.conf
echo "/usr/lib/jvm/java-7-oracle/jre/lib/amd64/server" | sudo tee --append /etc/ld.so.conf.d/jvm.conf
sudo ldconfig

# Install GeoWave as a service and configure to run at startup
# Note: tr removes carriage returns and copies the file
sudo tr -d '\r' < /vagrant/scripts/linux-install-scripts/geowave > /etc/init.d/geowave
sudo chmod 755 /etc/init.d/geowave
sudo update-rc.d geowave defaults
sudo service geowave start
