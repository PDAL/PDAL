#!/bin/bash -e
# Installs GeoWave library
git clone https://github.com/ngageoint/geowave.git geowave
cd geowave
git checkout tags/v0.8.5
mvn clean package -pl geowave-deploy -am -P generate-jace-proxies,linux-amd64-gcc -DskipITs=true -DskipTests=true

# Configure library paths
chmod 777 /home/vagrant/geowave/geowave-deploy/target/dependency/jace/libjace.so
sudo ln -s /home/vagrant/geowave/geowave-deploy/target/dependency/jace/libjace.so /usr/lib/libjace.so
echo "/usr/lib/jvm/java-7-oracle/jre/lib/amd64" | sudo tee --append /etc/ld.so.conf.d/awt.conf
echo "/usr/lib/jvm/java-7-oracle/jre/lib/amd64/server" | sudo tee --append /etc/ld.so.conf.d/jvm.conf
sudo ldconfig

# Install GeoWave as a service and configure to run at startup
# Note: tr removes carriage returns and copies the file
sudo tr -d '\r' < /vagrant/scripts/linux-install-scripts/geowave > /etc/init.d/geowave
sudo chmod 755 /etc/init.d/geowave
sudo update-rc.d geowave defaults
sudo service geowave start
