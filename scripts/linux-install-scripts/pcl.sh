NUMTHREADS=2
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS


git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build
cd build
git fetch origin --tags
git checkout tags/pcl-1.7.2
cmake .. \
    -G "Unix Makefiles" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr \
    -DBUILD_outofcore:BOOL=OFF \
    -DWITH_QT:BOOL=ON \
    -DWITH_VTK:BOOL=OFF \
    -DWITH_OPENNI:BOOL=OFF \
    -DWITH_CUDA:BOOL=OFF \
    -DWITH_LIBUSB:BOOL=OFF \
    -DBUILD_people:BOOL=OFF \
    -DBUILD_surface:BOOL=ON \
    -DBUILD_tools:BOOL=OFF \
    -DBUILD_visualization:BOOL=OFF \
    -DBUILD_sample_consensus:BOOL=ON \
    -DBUILD_tracking:BOOL=OFF \
    -DBUILD_stereo:BOOL=OFF \
    -DBUILD_keypoints:BOOL=OFF \
    -DBUILD_pipeline:BOOL=ON \
    -DCMAKE_CXX_FLAGS="-std=c++11" \
    -DBUILD_io:BOOL=ON \
    -DBUILD_octree:BOOL=ON \
    -DBUILD_segmentation:BOOL=ON \
    -DBUILD_search:BOOL=ON \
    -DBUILD_geometry:BOOL=ON \
    -DBUILD_filters:BOOL=ON \
    -DBUILD_features:BOOL=ON \
    -DBUILD_kdtree:BOOL=ON \
    -DBUILD_common:BOOL=ON \
    -DBUILD_ml:BOOL=ON

make -j $NUMTHREADS
sudo make install

