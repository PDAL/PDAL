@echo off

set COMPILER="Visual Studio 10 Win64"
set COMPILER="Visual Studio 10"

set BUILD_TYPE=Release
set BUILD_TYPE=Debug

set UTILS_DIR=c:\Utils
set DEV_DIR=c:\dev
set OSGEO4W_DIR=C:\OSGeo4W

rem *** These packages are needed from OSGeo4W
rem ***  gdal
rem ***  libxml2
rem ***  iconv  (might be pulled in automatically by libxml2?)
rem ***  oci


set BOOST_DIR=%UTILS_DIR%\boost_pro_1_46_1
set FREEGLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp
set GDAL_DIR=%DEV_DIR%\gdal
set GLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp
set ICONV_DIR=%UTILS_DIR%\iconv-1.9.2.win32
set LASZIP_DIR=%DEV_DIR%\laszip
set LIBLAS_DIR=%DEV_DIR%\liblas
set LIBXML2_DIR=%UTILS_DIR%\libxml2-2.7.7.win32
set PDAL_DIR=%DEV_DIR%\pdal

set ORACLE_HOME=%OSGEO4W_DIR%

cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=ON ^
    -DWITH_GEOTIFF=ON ^
    -DWITH_ORACLE=ON ^
    -DWITH_LASZIP=ON ^
    -DWITH_LIBLAS=ON ^
    -DWITH_FREEGLUT=ON ^
    -DWITH_LIBXML2=ON ^
    -DWITH_ICONV=OFF ^
    -DFREEGLUT_LIBRARY=%FREEGLUT_DIR%\lib\freeglut.lib ^
    -DFREEGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DGLUT_LIBRARY=%FREEGLUT_DIR%\lib ^
    -DGLUT_INCLUDE_DIR=%FREEGLUT_DIR%\include ^
    -DTIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DTIFF_LIBRARY=%OSGEO4W_DIR%\lib\libtiff_i.lib ^
    -DGEOTIFF_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DGEOTIFF_LIBRARY=%OSGEO4W_DIR%\lib\geotiff_i.lib ^
    -DGDAL_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DGDAL_LIBRARY=%OSGEO4W_DIR%\lib\gdal_i.lib ^
    -DORACLE_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DORACLE_OCI_LIBRARY=%OSGEO4W_DIR%\lib\oci.lib ^
    -DLASZIP_INCLUDE_DIR=%LASZIP_DIR%\include ^
    -DLASZIP_LIBRARY=%LASZIP_DIR%\bin\Debug\Debug\laszip.lib ^
    -DLIBLAS_INCLUDE_DIR=%LIBLAS_DIR%\include ^
    -DLIBLAS_LIBRARY=%LIBLAS_DIR%\bin\Debug\Debug\liblas.lib ^
    -DLIBXML2_LIBRARIES=%OSGEO4W_DIR%\lib\libxml2.lib ^
    -DLIBXML2_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %PDAL_DIR%

rem *** for dev build of gdal ***
rem    -DGDAL_INCLUDE_DIR=%GDAL_DIR%\gcore ^
rem    -DGDAL_LIBRARY=%GDAL_DIR%\gdal_i.lib ^
rem    -DTIFF_INCLUDE_DIR=%GDAL_DIR%\frmts\gtiff\libtiff ^
rem    -DTIFF_LIBRARY=%GDAL_DIR%\gdal_i.lib ^
rem    -DGEOTIFF_INCLUDE_DIR=%GDAL_DIR%\frmts\gtiff\libgeotiff ^
rem    -DGEOTIFF_LIBRARY=%GDAL_DIR%\gdal_i.lib ^

rem    -DLIBXML2_LIBRARIES=%LIBXML2_DIR%\lib\libxml2.lib ^
rem    -DLIBXML2_INCLUDE_DIR=%LIBXML2_DIR%\include ^
rem    -DICONV_LIBRARY=%ICONV_DIR%\lib\iconv.lib ^
rem    -DICONV_INCLUDE_DIR=%ICONV_DIR%\include ^

set PDAL_SWIG_BOOST_HOME=%UTILS_DIR%\boost_1_45_0-win32
