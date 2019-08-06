NUMTHREADS=2
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS
export JAVA_HOME=/usr/lib/jvm/java-7-oracle

git clone https://github.com/PDAL/PDAL.git pdal
cd pdal
mkdir build
cd build
cmake   -G "Unix Makefiles"  \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr \
        -DWITH_ICONV=ON \
        -DWITH_LASZIP=ON \
        -DWITH_LAZPERF=ON \
        -DWITH_LIBXML2=ON \
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DLAZPERF_INCLUDE_DIR=/home/vagrant/laz-perf \
        -DBUILD_PLUGIN_GEOWAVE=ON \
        -DGEOWAVE_RUNTIME_JAR=/home/vagrant/geowave/geowave-jace.jar \
        -DJACE_INCLUDE_DIR=/home/vagrant/geowave/include \
        -DJACE_LIBRARY=/home/vagrant/geowave/build/libjace.so \
        ..

make -j $NUMTHREADS
sudo make install

