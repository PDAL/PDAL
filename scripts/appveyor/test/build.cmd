call "%CONDA_ROOT%\Scripts\activate.bat" base
call conda install geotiff laszip nitro curl gdal cmake eigen ninja libgdal zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 tiledb conda-build ninja -y

echo "before vcvars"
dir
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64

echo "after vcvars"

SET PDAL_INSTALL_PREFIX="C:/projects/pdal/install"
SET PDAL_PLUGIN_INSTALL_PATH="C:/projects/pdal/build/bin"
SET PDAL_BUILD_TESTS=ON
SET CMAKE_VERBOSE_MAKEFILE=OFF
SET CMAKE_BUILD_TYPE=RelWithDebInfo

mkdir build
pushd build

set ORACLE_HOME=%CONDA_ROOT%

cmake -G "Ninja" ^
    -DCMAKE_BUILD_TYPE=%CMAKE_BUILD_TYPE% ^
	-DPDAL_PLUGIN_INSTALL_PATH:FILEPATH=%PDAL_PLUGIN_INSTALL_PATH% ^
    -DWITH_TESTS=%PDAL_BUILD_TESTS% ^
    -DCMAKE_VERBOSE_MAKEFILE=%CMAKE_VERBOSE_MAKEFILE% ^
    -DCMAKE_LIBRARY_PATH:FILEPATH="%CONDA_ROOT%/Library/lib" ^
    -DCMAKE_INCLUDE_PATH:FILEPATH="%CONDA_ROOT%/Library/include" ^
    -DOPENSSL_ROOT_DIR=%CONDA_ROOT%/Library ^
    -DPython3_ROOT_DIR:FILEPATH="%CONDA_PREFIX%" ^
    -DBUILD_PLUGIN_CPD=OFF ^
    -DBUILD_PLUGIN_GREYHOUND=ON ^
    -DBUILD_PLUGIN_ICEBRIDGE=ON ^
    -DBUILD_PLUGIN_MRSID=OFF ^
    -DBUILD_PLUGIN_NITF=ON ^
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
    -DBUILD_PLUGIN_OCI=OFF ^
    -DBUILD_PLUGIN_SQLITE=ON ^
    -DBUILD_PLUGIN_I3S=ON ^
    -DBUILD_PLUGIN_RIVLIB=OFF ^
    -DBUILD_PLUGIN_PYTHON=ON ^
    -DENABLE_CTEST=OFF ^
    -DWITH_LAZPERF=ON ^
    -DWITH_LZMA=ON ^
    -DLIBLZMA_LIBRARY:FILEPATH=%CONDA_ROOT%\Library\lib\liblzma.lib ^
    -DZSTD_LIBRARY:FILEPATH=%CONDA_ROOT%\Library\lib\libzstd.lib ^
    -DWITH_LASZIP=ON ^
    -DORACLE_INCLUDE_DIR=%CONDA_ROOT%/include ^
    -DORACLE_LIBRARY=%CONDA_ROOT%/libs/oci.lib ^
    -DLazperf_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DHDF5_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DLazperf_DIR:FILEPATH=%CONDA_ROOT%/Library/cmake ^
    -DWITH_ZLIB=ON ^
    -Dgtest_force_shared_crt=ON ^
    -DBUILD_PGPOINTCLOUD_TESTS=OFF ^
    -DBUILD_SQLITE_TESTS=OFF ^
    -DBUILD_I3S_TESTS=OFF ^
    -DBUILD_OCI_TESTS=OFF ^
    ..




REM nmake /f Makefile
ninja
popd
