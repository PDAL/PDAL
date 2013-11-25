NUMTHREADS=2
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS


git clone https://github.com/PDAL/PDAL.git pdal
cd pdal
sudo git checkout master
cmake   -G "Unix Makefiles"  \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DPDAL_EMBED_BOOST=OFF \
        -DWITH_GDAL=ON \
        -DWITH_ICONV=ON \
        -DWITH_GEOTIFF=ON \
        -DWITH_LASZIP=ON \
        -DWITH_LIBXML2=ON \
        -DWITH_PYTHON=ON \
        -DWITH_SQLITE=ON \
        -DWITH_P2G=ON \
        -DWITH_HEXER=ON \
        -DWITH_NITRO=ON \
        -DWITH_PGPOINTCLOUD=ON

make -j $NUMTHREADS
sudo make install

