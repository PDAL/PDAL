@echo off

set COMPILER="Visual Studio 10 Win64"

set LIBPC_DIR=c:\dev\libpc

set LASZIP_DIR=c:\dev\laszip
set OSGEO4W_DIR=C:\OSGeo4W
set OSGEO4W_GDAL=C:\OSGeo4W\apps\gdal-17
set BOOST_DIR="C:\dev\boost_1_45_0"
set ORACLE_HOME=%OSGEO4W_DIR%

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=OFF ^
    -DWITH_GEOTIFF=OFF ^
    -DWITH_ORACLE=OFF ^
    -DWITH_LASZIP=ON ^
    -DGDAL_INCLUDE_DIR=%OSGEO4W_GDAL%\include ^
    -DGDAL_LIBRARY=%OSGEO4W_GDAL%\lib\gdal_i.lib ^
    -DTIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DTIFF_LIBRARY=%OSGEO4W_DIR%\lib\libtiff_i.lib ^
    -DGEOTIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DGEOTIFF_LIBRARY=%OSGEO4W_DIR%\lib\geotiff_i.lib ^
    -DORACLE_INCLUDE_DIR=%ORACLE_HOME%\include ^
    -DORACLE_OCI_LIBRARY=%ORACLE_HOME%\lib\oci.lib ^
    -DLASZIP_INCLUDE_DIR=%LASZIP_DIR%\include ^
    -DLASZIP_LIBRARY=%LASZIP_DIR%\bin\Debug\Debug\laszip.lib ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %LIBPC_DIR%
