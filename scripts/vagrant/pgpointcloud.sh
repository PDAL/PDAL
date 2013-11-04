git clone https://github.com/pramsey/pointcloud.git
cmake -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS="-fPIC" \
    -DCMAKE_C_FLAGS="-fPIC"
make
sudo make install
sudo -u postgres createuser -s vagrant
createdb points
echo "CREATE EXTENSION pointcloud;" | psql -d points -U vagrant
wget http://liblas.org/samples/st-helens-small.las
https://raw.github.com/PDAL/PDAL/master/scripts/vagrant/loadpgpointcloud.xml
pdal pipeline --input loadpgpointcloud.xml
