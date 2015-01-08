NUMTHREADS=2
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS


git clone https://github.com/PDAL/PDAL.git pdal
cd pdal
mkdir build
cd build
cmake   -G "Unix Makefiles"  \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DWITH_ICONV=ON \
        -DBUILD_PLUGIN_PCL=ON \
        -DWITH_LASZIP=ON \
        -DWITH_GEOTIFF=ON \
        -DWITH_LAZPERF=ON \
        -DWITH_LIBXML2=ON \
        -DBUILD_PLUGIN_ATTRIBUTE=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_P2G=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        ..

make -j $NUMTHREADS
sudo make install

