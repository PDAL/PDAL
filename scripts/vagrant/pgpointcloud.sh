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
createdb points
echo "CREATE EXTENSION postgis;" | psql -d points -U vagrant
echo "CREATE EXTENSION pointcloud;" | psql -d points -U vagrant
echo "CREATE EXTENSION pointcloud_postgis;" | psql -d points -U vagrant
wget http://liblas.org/samples/st-helens-small.las
wget https://raw.github.com/PDAL/PDAL/master/scripts/vagrant/loadpgpointcloud.xml
wget https://raw.github.com/PDAL/PDAL/master/scripts/vagrant/readpgpointcloud.xml
pdal pipeline --input loadpgpointcloud.xml
pdal info --input readpgpointcloud.xml -p 0
