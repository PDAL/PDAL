@echo off

:: Pick your architecture, 32 or 64 bits 
rem set COMPILER="NMake Makefiles"
rem set COMPILER="Visual Studio 10 Win64"
set COMPILER="Visual Studio 10"


:: Pick your build type
rem set BUILD_TYPE=Release
set BUILD_TYPE=Debug


:: Set some useful path variables
::    "utils" is where you might have libraries installed, like Boost
::    "dev" is where you build things yourself, like PDAL and laszip
set UTILS_DIR=d:\utils
set DEV_DIR=d:\dev


:: Where is your OSGeo4W installed?
set OSGEO4W_DIR=C:\OSGeo4W


:: Where is boost installed?
set BOOST_DIR=%UTILS_DIR%\boost_1_47


:: Where are the GLUT libs installed?
::  (only needed if you want to build pcview)
set FREEGLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp
set GLUT_DIR=%DEV_DIR%\freeglut-2.6.0-3.mp

:: Where is LASZIP?  (can be either from OSGeo4W or your own build tree)
rem  set LASZIP_LIBRARY=%OSGEO4W_DIR%\laszip\laszip.lib
rem  set LASZIP_INCLUDE_DIR=%OSGEO4W_DIR%\laszip\include
set LASZIP_LIBRARY=%DEV_DIR%\laszip\bin\Debug\Debug\laszip.lib
set LASZIP_INCLUDE_DIR=%DEV_DIR%\laszip\include


:: Where is LIBLAS?  (can be either from OSGeo4W or your own build tree)
rem  set LIBLAS_LIBRARY=%OSGEO4W_DIR%\lib\liblas.lib
rem  set LIBLAS_INCLUDE_DIR=%OSGEO4W_DIR%\include
rem set LIBLAS_LIBRARY=%DEV_DIR%\liblas\bin\Debug\Debug\liblas.lib
rem set LIBLAS_INCLUDE_DIR=%DEV_DIR%\liblas\include


:: Where is your PDAL build tree?
set PDAL_DIR=%DEV_DIR%\pdal


:: Where are the OCI libraries installed?
set ORACLE_HOME=%OSGEO4W_DIR%


:: Set this if you are building SWIG bindings for C#.  Visual Studio
:: needs to use this env var to find where boost lives.
set PDAL_SWIG_BOOST_HOME=%BOOST_DIR%


:: If you set the above stuff correctly, then you should only need to
:: modify the "WITH" (on/off) settings in the following CMAKE invocation.
::
:: For most people, you should have these turned ON:
::    GDAL
::    GEOTIFF
::    ORACLE
::    LASZIP
::    LIBXML2
:: and these turned OFF:
::    LIBLAS
::    FREEGLUT
::    SWIG_CSHARP
cmake -G %COMPILER% ^
    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
    -DWITH_GDAL=ON ^
    -DWITH_GEOTIFF=ON ^
    -DWITH_ORACLE=ON ^
    -DWITH_LASZIP=ON ^
    -DWITH_LIBLAS=OFF ^
    -DWITH_FREEGLUT=ON ^
    -DWITH_LIBXML2=ON ^
    -DWITH_SWIG_CSHARP=ON ^
	-DWITH_PLANG=ON ^
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
    -DLASZIP_INCLUDE_DIR=%LASZIP_INCLUDE_DIR% ^
    -DLASZIP_LIBRARY=%LASZIP_LIBRARY% ^
    -DLIBXML2_LIBRARIES=%OSGEO4W_DIR%\lib\libxml2.lib ^
    -DLIBXML2_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
	-DICONV_LIBRARY=%OSGEO4W_DIR%\lib\iconv.lib ^
	-DICONV_INCLUDE_DIR=%OSGEO4W_DIR%\include ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %PDAL_DIR%

rem     -DLIBLAS_INCLUDE_DIR=%LIBLAS_INCLUDE_DIR% ^
rem     -DLIBLAS_LIBRARY=%LIBLAS_LIBRARY% ^
