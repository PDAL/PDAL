@echo off

set G="Visual Studio 10 Win64"
set LIBPC=d:\dev\libpc
set LASZIP=d:\dev\laszip
set OSGEO4W=C:\OSGeo4W
set OSGEO4W_GDAL=C:\OSGeo4W\apps\gdal-17
set BOOST="C:\Utils\boost_1_45_0"
REM set ORACLE_HOME=%OSGEO4W%

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

cmake -G %G% ^
    -DBOOST_INCLUDEDIR=%BOOST% ^
    -DWITH_GDAL=OFF ^
    -DWITH_GEOTIFF=OFF ^
    -DWITH_ORACLE=OFF ^
    -DWITH_LASZIP=ON ^
    -DGDAL_INCLUDE_DIR=%OSGEO4W_GDAL%\include ^
    -DGDAL_LIBRARY=%OSGEO4W_GDAL%\lib\gdal_i.lib ^
    -DTIFF_INCLUDE_DIR=%OSGEO4W%\include ^
    -DTIFF_LIBRARY=%OSGEO4W%\lib\libtiff_i.lib ^
    -DGEOTIFF_INCLUDE_DIR=%OSGEO4W%\include ^
    -DGEOTIFF_LIBRARY=%OSGEO4W%\lib\geotiff_i.lib ^
    -DORACLE_INCLUDE_DIR=%ORACLE_HOME%\include ^
    -DORACLE_OCI_LIBRARY=%ORACLE_HOME%\lib\oci.lib ^
    -DLASZIP_INCLUDE_DIR=%LASZIP%\include ^
    -DLASZIP_LIBRARY=%LASZIP%\bin\Debug\Debug\laszip.lib ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %LIBPC%
