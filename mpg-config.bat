@echo off

set COMPILER="Visual Studio 10 Win64"
set COMPILER="Visual Studio 10"

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

set UTILS_DIR=c:\Utils
set DEV_DIR=c:\dev

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

set ICONV_DIR=%UTILS_DIR%\iconv-1.9.2.win32

cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=OFF ^
    -DWITH_GEOTIFF=OFF ^
    -DWITH_ORACLE=OFF ^
    -DWITH_LASZIP=ON ^
    -DWITH_LIBLAS=ON ^
    -DWITH_FREEGLUT=ON ^
    -DWITH_LIBXML2=ON ^
    -DWITH_ICONV=ON ^
    -DLIBXML2_LIBRARIES=%LIBXML2_DIR%\lib\libxml2.lib ^
    -DLIBXML2_INCLUDE_DIR=%LIBXML2_DIR%\include ^
    -DICONV_LIBRARY=%ICONV_DIR%\lib\iconv.lib ^
    -DICONV_INCLUDE_DIR=%ICONV_DIR%\include ^
    -DFREEGLUT_LIBRARY=%FREEGLUT_DIR%\lib\freeglut.lib ^
    -DFREEGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DGLUT_LIBRARY=%FREEGLUT_DIR%\lib ^
    -DGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DLASZIP_INCLUDE_DIR=%LASZIP_DIR%\include ^
    -DLASZIP_LIBRARY=%LASZIP_DIR%\bin\Debug\Debug\laszip.lib ^
    -DLIBLAS_INCLUDE_DIR=%LIBLAS_DIR%\include ^
    -DLIBLAS_LIBRARY=%LIBLAS_DIR%\bin\Debug\Debug\liblas.lib ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %PDAL_DIR%

set PDAL_SWIG_BOOST_HOME=%UTILS_DIR%\boost_1_45_0-win32
