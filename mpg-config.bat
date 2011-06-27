@echo off

set COMPILER="Visual Studio 10 Win64"
set COMPILER="Visual Studio 10"

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

set UTILS_DIR=c:\Utils
set DEV_DIR=d:\dev

set PDAL_DIR=%DEV_DIR%\pdal

set LIBLAS_DIR=%DEV_DIR%\liblas
set LASZIP_DIR=%DEV_DIR%\laszip
set OSGEO4W_DIR=C:\OSGeo4W
set GDAL_DIR=%DEV_DIR%\gdal
set BOOST_DIR=%UTILS_DIR%\boost_pro_1_46_1
set ORACLE_HOME=%OSGEO4W_DIR%

set FREEGLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp
set GLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp

set LIBXML2_DIR=%UTILS_DIR%\libxml2-2.7.7.win32

cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=ON ^
    -DWITH_GEOTIFF=ON ^
    -DWITH_ORACLE=ON ^
    -DWITH_LASZIP=ON ^
    -DWITH_LIBLAS=ON ^
    -DWITH_FREEGLUT=ON ^
    -DWITH_LIBXML2=ON ^
    -DLIBXML2_LIBRARIES=%LIBXML2_DIR%\lib\libxml2.lib ^
    -DLIBXML2_INCLUDE_DIR=%LIBXML2_DIR%\include ^
    -DFREEGLUT_LIBRARY=%FREEGLUT_DIR%\lib\freeglut.lib ^
    -DFREEGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DGLUT_LIBRARY=%FREEGLUT_DIR%\lib ^
    -DGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DGDAL_INCLUDE_DIR=%GDAL_DIR%\gcore ^
    -DGDAL_LIBRARY=%GDAL_DIR%\gdal_i.lib ^
    -DTIFF_INCLUDE_DIR=%GDAL_DIR%\frmts\gtiff\libtiff ^
    -DTIFF_LIBRARY=%GDAL_DIR%\gdal_i.lib ^
    -DGEOTIFF_INCLUDE_DIR=%GDAL_DIR%\frmts\gtiff\libgeotiff ^
    -DGEOTIFF_LIBRARY=%GDAL_DIR%\gdal_i.lib ^
    -DORACLE_INCLUDE_DIR=%ORACLE_HOME%\include ^
    -DORACLE_OCI_LIBRARY=%ORACLE_HOME%\lib\oci.lib ^
    -DLASZIP_INCLUDE_DIR=%LASZIP_DIR%\include ^
    -DLASZIP_LIBRARY=%LASZIP_DIR%\bin\Debug\Debug\laszip.lib ^
    -DLIBLAS_INCLUDE_DIR=%LIBLAS_DIR%\include ^
    -DLIBLAS_LIBRARY=%LIBLAS_DIR%\bin\Debug\Debug\liblas.lib ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %PDAL_DIR%

set PDAL_SWIG_BOOST_HOME=%UTILS_DIR%\boost_1_45_0-win32
