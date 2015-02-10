git clone https://github.com/pramsey/pointcloud.git
cd pointcloud
cmake -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-fPIC" \
    -DCMAKE_C_FLAGS="-fPIC"
make
sudo make install
sudo service postgresql start
sudo -u postgres createuser -s vagrant
sudo -u postgres createdb points
echo "CREATE EXTENSION postgis;" | sudo -u vagrant psql -d points -U vagrant
echo "CREATE EXTENSION pointcloud;" | sudo -u vagrant psql -d points -U vagrant
echo "CREATE EXTENSION pointcloud_postgis;" | sudo -u vagrant psql -d points -U vagrant
sudo wget http://liblas.org/samples/st-helens-small.las
sudo wget https://raw.github.com/PDAL/PDAL/master/scripts/vagrant/loadpgpointcloud.xml
sudo wget https://raw.github.com/PDAL/PDAL/master/scripts/vagrant/readpgpointcloud.xml
chmod 777 st-helens-small.las
chmod 777 readpgpointcloud.xml
chmod 777 loadpgpointcloud.xml
sudo -u vagrant PDAL_DRIVER_PATH=/usr/lib pdal pipeline --input loadpgpointcloud.xml
sudo -u vagrant PDAL_DRIVER_PATH=/usr/lib pdal info --input readpgpointcloud.xml -p 0
