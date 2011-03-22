@echo off

set COMPILER="Visual Studio 10 Win64"
set COMPILER="Visual Studio 10"

set LIBPC_DIR=d:\dev\libpc

set LIBLAS_DIR=d:\dev\liblas
set LASZIP_DIR=d:\dev\laszip
set OSGEO4W_DIR=C:\OSGeo4W
set OSGEO4W_GDAL=C:\OSGeo4W\apps\gdal-17
set BOOST_DIR="C:\Utils\boost_1_45_0-win32"
set ORACLE_HOME=%OSGEO4W_DIR%

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=ON ^
    -DWITH_GEOTIFF=ON ^
    -DWITH_ORACLE=ON ^
    -DWITH_LASZIP=ON ^
    -DWITH_LIBLAS=ON ^
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
    -DLIBLAS_INCLUDE_DIR=%LIBLAS_DIR%\include ^
    -DLIBLAS_LIBRARY=%LIBLAS_DIR%\bin\Debug\Debug\liblas.lib ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %LIBPC_DIR%

set LIBPC_SWIG_BOOST_HOME=C:\Utils\boost_1_45_0-win32
