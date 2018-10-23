@echo off

REM If OSGEO4W_BUILD is set, we build an OSGeo4W64 tarball package and
REM and install it to C:\pdalbin before letting AppVeyor upload it as
REM an artifact to S3.

SET PDAL_INSTALL_PREFIX="C:/temp/install"
SET PDAL_PLUGIN_INSTALL_PATH="C:/temp/install/bin"
set PDAL_BUILD_TESTS=ON

REM needed or else CMake won't find the Oracle library that OSGeo4W installs


mkdir build
pushd build

cmake -G "NMake Makefiles" ^
    -DBUILD_PLUGIN_CPD=OFF ^
    -DBUILD_PLUGIN_GREYHOUND=ON ^
    -DBUILD_PLUGIN_HEXBIN=ON ^
    -DBUILD_PLUGIN_ICEBRIDGE=OFF ^
    -DBUILD_PLUGIN_MRSID=OFF ^
    -DBUILD_PLUGIN_NITF=ON ^
    -DBUILD_PLUGIN_PCL=ON ^
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
    -DBUILD_PLUGIN_SQLITE=ON ^
    -DBUILD_PLUGIN_I3S=ON ^
    -DLIBLZMA_LIBRARY=%CONDA_ROOT%\Library\lib\liblzma.lib ^
    -DBUILD_PLUGIN_RIVLIB=OFF ^
    -DBUILD_PLUGIN_PYTHON=ON ^
    -DENABLE_CTEST=OFF ^
    -DWITH_LAZPERF=ON ^
    -DLazperf_DIR=%CONDA_ROOT% ^
    -DWITH_LZMA=ON ^
    -DWITH_LASZIP=ON ^
    -DWITH_TESTS=ON ^
	-DPDAL_PLUGIN_INSTALL_PATH=%PDAL_PLUGIN_INSTALL_PATH% ^
	-DGDAL_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DGDAL_LIBRARY=%CONDA_ROOT%/Library/lib/gdal_i.lib ^
	-DGEOTIFF_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DGEOTIFF_LIBRARY=%CONDA_ROOT%/Library/lib/geotiff_i.lib ^
	-DGEOS_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DGEOS_LIBRARY=%CONDA_ROOT%/Library/lib/geos.lib ^
	-DZLIB_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DZLIB_LIBRARY=%CONDA_ROOT%/Library/lib/zlib.lib ^
	-DCURL_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DCURL_LIBRARY=%CONDA_ROOT%/Library/lib/libcurl.lib ^
	-DLIBXML2_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DLIBXML2_LIBRARY=%CONDA_ROOT%/Library/lib/libxml2_a.lib ^
	-DNITRO_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DNITRO_CPP_LIBRARY=%CONDA_ROOT%/Library/lib/nitf-cpp.lib ^
	-DNITRO_C_LIBRARY=%CONDA_ROOT%/Library/lib/nitf-c.lib ^
	-DEIGEN_INCLUDE_DIRS=%CONDA_ROOT%/Library/include ^
	-DPOSTGRESQL_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DPOSTGRESQL_LIBRARIES=%CONDA_ROOT%/Library/lib/postgres.lib ^
	-DSQLITE3_INCLUDE_DIR=%CONDA_ROOT%/Library/include ^
	-DSQLITE3_LIBRARY=%CONDA_ROOT%/Library/lib/sqlite3.lib ^
    -DLazperf_DIR=%CONDA_ROOT%/Library/cmake ^
    -DHDF5_DIR=%CONDA_ROOT%/Library/cmake ^
    -DPCL_DIR=%CONDA_ROOT%/Library/cmake ^
    -DWITH_ZLIB=ON ^
	-DPYTHON_EXECUTABLE=%CONDA_ROOT%/python.exe ^
	-DPYTHON_LIBRARY=%CONDA_ROOT%/libs/python37.lib ^
	-DPYTHON_DEBUG_LIBRARY=%CONDA_ROOT%libs/python37.lib ^
    -Dgtest_force_shared_crt=ON ^
    -DCMAKE_INSTALL_PREFIX=%PDAL_INSTALL_PREFIX% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    -DBUILD_PGPOINTCLOUD_TESTS=OFF ^
    -DBUILD_SQLITE_TESTS=OFF ^
    -DCMAKE_BUILD_TYPE=RelWithDebInfo ^
    -DBUILD_OCI_TESTS=OFF ^
    ..

popd

