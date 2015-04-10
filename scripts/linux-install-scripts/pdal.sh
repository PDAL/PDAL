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
        -DBUILD_PLUGIN_PYTHON=ON \
        -DBUILD_PLUGIN_ATTRIBUTE=ON \
        -DBUILD_PLUGIN_HEXBIN=ON \
        -DBUILD_PLUGIN_ICEBRIDGE=ON \
        -DBUILD_PLUGIN_NITF=ON \
        -DBUILD_PLUGIN_P2G=ON \
        -DBUILD_PLUGIN_PGPOINTCLOUD=ON \
        -DBUILD_PLUGIN_SQLITE=ON \
        -DBUILD_PLUGIN_GREYHOUND=ON \
        -DLAZPERF_INCLUDE_DIR=/home/vagrant/laz-perf \
        -DJSONCPP_ROOT_DIR=/usr/include/jsoncpp \
        -DBUILD_PLUGIN_GEOWAVE=ON \
        -DGEOWAVE_RUNTIME_JAR=/home/vagrant/geowave/geowave-deploy/target/geowave-deploy-0.8.5-jace.jar \
        -DJACE_INCLUDE_DIR=/home/vagrant/geowave/geowave-deploy/target/dependency/jace/include \
        -DJACE_LIBRARY=/home/vagrant/geowave/geowave-deploy/target/dependency/jace/libjace.so \
        -DJACE_RUNTIME_JAR=/home/vagrant/geowave/geowave-deploy/target/dependency/jace-core-runtime-1.2.22.jar \
        -DJAVA_AWT_INCLUDE_PATH=/usr/lib/jvm/java-7-oracle/include \
        -DJAVA_AWT_LIBRARY=/usr/lib/jvm/java-7-oracle/jre/lib/amd64/libjawt.so \
        -DJAVA_INCLUDE_PATH=/usr/lib/jvm/java-7-oracle/include \
        -DJAVA_INCLUDE_PATH2=/usr/lib/jvm/java-7-oracle/include/linux \
        -DJAVA_JVM_LIBRARY=/usr/lib/jvm/java-7-oracle/jre/lib/amd64/server/libjvm.so \
        ..

make -j $NUMTHREADS
sudo make install

