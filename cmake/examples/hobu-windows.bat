@echo off

:: This configure script is designed for the default Windows world, which means
:: you have OSGeo4W installed, including Oracle and GDAL and LASzip.

:: This configure script expects to be run from the PDAL root directory.

:: Pick your CMake GENERATOR.  (NMake will pick up architecture (x32, x64) from your environment)
set GENERATOR="NMake Makefiles"
REM set GENERATOR="Ninja"
REM set GENERATOR="Visual Studio 14 Win64"

:: Pick your build type
set BUILD_TYPE=Release
REM set BUILD_TYPE=Debug

:: Where is your PDAL build tree?
set PDAL_DIR=.

:: Where is your OSGeo4W installed (recommended basic way to satisfy dependent libs)
set OSGEO4W_DIR=C:\OSGeo4W64

:: GDAL
set GDAL_ENABLED=ON
set GDAL_INCLUDE_DIR=%OSGEO4W_DIR%\include
set GDAL_LIBRARY=%OSGEO4W_DIR%\lib\gdal_i.lib

:: LIBTIFF
set TIFF_ENABLED=ON
set TIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include
set TIFF_LIBRARY=%OSGEO4W_DIR%\lib\libtiff_i.lib

:: GeoTIFF
set GEOTIFF_ENABLED=ON
set GEOTIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include
set GEOTIFF_LIBRARY=%OSGEO4W_DIR%\lib\geotiff_i.lib

:: LASZIP
set LASZIP_ENABLED=ON
set LASZIP_INCLUDE_DIR=%OSGEO4W_DIR%\include
set LASZIP_LIBRARY=%OSGEO4W_DIR%\lib\laszip.lib

:: Oracle
set ORACLE_ENABLED=ON
set ORACLE_HOME=%OSGEO4W_DIR%
set ORACLE_INCLUDE_DIR=%ORACLE_HOME%\include
set ORACLE_OCI_LIBRARY=%ORACLE_HOME%\lib\oci.lib

:: LibXML2
set LIBXML2_INCLUDE_DIR=%OSGEO4W_DIR%\include
set LIBXML2_LIBRARIES=%OSGEO4W_DIR%\lib\libxml2.lib

:: Python
set PYTHON_ENABLED=ON
set PYTHON_EXECUTABLE=%OSGEO4W_DIR\bin\python36.exe
set PYTHON_INCLUDE_DIR=%OSGEO4W_DIR\apps\python36\include
set PYTHON_LIBRARY=%OSGEO4W_DIR\apps\python36\libs\python36.lib


:: CURL
set CURL_INCLUDE_DIR=%OSGEO4W_DIR%\include
set CURL_LIBRARY=%OSGEO4W_DIR%\lib\libcurl.lib


if EXIST CMakeCache.txt del CMakeCache.txt

cmake -G %GENERATOR% ^
    -DBUILD_PLUGIN_CPD=OFF ^
    -DBUILD_PLUGIN_GREYHOUND=ON ^
    -DBUILD_PLUGIN_ICEBRIDGE=OFF ^
    -DBUILD_PLUGIN_MRSID=OFF ^
    -DBUILD_PLUGIN_NITF=ON ^
    -DBUILD_PLUGIN_OCI=ON ^
    -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
    -DBUILD_PLUGIN_SQLITE=ON ^
    -DBUILD_PLUGIN_I3S=ON ^
    -DBUILD_PLUGIN_RIVLIB=OFF ^
    -DBUILD_PLUGIN_PYTHON=ON ^
    -DENABLE_CTEST=OFF ^
    -DWITH_LAZPERF=ON ^
    -DLazperf_DIR=%OSGEO4W_DIR%/cmake ^
    -DWITH_LASZIP=ON ^
    -DWITH_TESTS=ON ^
    -DPYTHON_EXECUTABLE=%OSGEO4W_DIR%\apps\python37\python.exe ^
	-DLIBLZMA_LIBRARY=%OSGEO4W_DIR%\lib\liblzma.lib ^
    -DPYTHON_INCLUDE_DIR=%OSGEO4W_DIR%\apps\python36\include ^
    -DPYTHON_LIBRARY=%OSGEO4W_DIR%\apps\python37\libs\python37.lib ^
    -DPYTHON_DEBUG_LIBRARY=%OSGEO4W_DIR%\apps\python37\libs\python37.lib ^
    -DCURL_INCLUDE_DIR=%CURL_INCLUDE_DIR% ^
    -DCURL_LIBRARY=%CURL_LIBRARY% ^
    -DNUMPY_INCLUDE_DIR=%OSGEO4W_DIR%\apps\python37\lib\site-packages\numpy\core\include ^
    -Dgtest_force_shared_crt=ON ^
    -DCMAKE_INSTALL_PREFIX=c:\OSGeo4W64\ ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF	 ^
    .

