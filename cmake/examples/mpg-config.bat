@echo off
 
:: This configure script is designed for the default Windows world, which means
:: you have OSGeo4W installed, including Oracle and GDAL and LASzip.

:: This configure script expects to be run from the PDAL root directory.

:: Pick your CMake GENERATOR.  (NMake will pick up architecture (x32, x64) from your environment)
rem set GENERATOR="NMake Makefiles"
set GENERATOR="Visual Studio 10 Win64"
set GENERATOR="Visual Studio 10"

:: Pick your build type
set BUILD_TYPE=Release
set BUILD_TYPE=Debug

:: Where is your PDAL build tree?
set PDAL_DIR=.

:: Where is your OSGeo4W installed (recommended basic way to satisfy dependent libs)
set OSGEO4W_DIR=C:\OSGeo4W

:: Where is boost installed?
rem set PDAL_EMBED_BOOST=OFF
rem set BOOST_DIR=c:\utils\boost_1_49_0
set BOOST_DIR=%PDAL_DIR%\boost

:: CARIS
set CARIS_ENABLED=ON
set CARIS_INCLUDE_DIR=%CARIS_DIR%\include
set CARIS_LIBRARY=%CARIS_DIR%\caris.lib

:: GDAL
set GDAL_ENABLED=ON
set GDAL_INCLUDE_DIR=%OSGEO4W_DIR%\apps\gdal-dev\include
set GDAL_LIBRARY=%OSGEO4W_DIR%\apps\gdal-dev\lib\gdal_i.lib

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
set LIBXML2_ENABLED=ON
set LIBXML2_INCLUDE_DIR=%OSGEO4W_DIR%\include
set LIBXML2_LIBRARIES=%OSGEO4W_DIR%\lib\libxml2.lib

:: IConv
set ICONV_ENABLED=ON
set ICONV_INCLUDE_DIR=%OSGEO4W_DIR%\include
set ICONV_LIBRARY=%OSGEO4W_DIR%\lib\iconv.lib

:: Python
set PYTHON_ENABLED=ON
set PYTHON_INCLUDE_DIR=c:\Utils\Python27\include
set PYTHON_LIBRARY=c:\Utils\Python27\libs\python27.lib

:: OpenGL support, for pcview
set FREEGLUT_ENABLED=OFF
:: special config for mpg
if %USERDOMAIN% == T5500 set FREEGLUT_LIBRARY=d:\dev\freeglut-2.6.0-3.mp\lib\freeglut.lib
if %USERDOMAIN% == T5500 set FREEGLUT_INCLUDE_DIR=d:\dev\freeglut-2.6.0-3.mp\include
if %USERDOMAIN% == T5500 set FREEGLUT_ENABLED=ON
if %USERDOMAIN% == PDC set FREEGLUT_LIBRARY=c:\dev\freeglut-2.6.0-3.mp\lib\freeglut.lib
if %USERDOMAIN% == PDC set FREEGLUT_INCLUDE_DIR=c:\dev\freeglut-2.6.0-3.mp\include
if %USERDOMAIN% == PDC set FREEGLUT_ENABLED=ON

rem if EXIST CMakeCache.txt del CMakeCache.txt
cmake -G %GENERATOR% ^
    -DWITH_CARIS=%CARIS_ENABLED% ^
    -DWITH_GDAL=%GDAL_ENABLED% ^
    -DWITH_GEOTIFF=%GEOTIFF_ENABLED% ^
    -DWITH_ORACLE=%ORACLE_ENABLED% ^
    -DWITH_LASZIP=%LASZIP_ENABLED% ^
    -DWITH_LIBXML2=%LIBXML2_ENABLED% ^
    -DWITH_ICONV=%ICONV_ENABLED% ^
	-DWITH_PYTHON=%PYTHON_ENABLED% ^
	-DWITH_FREEGLUT=%FREEGLUT_ENABLED% ^
	-DFREEGLUT_LIBRARY=%FREEGLUT_LIBRARY% ^
    -DFREEGLUT_INCLUDE_DIR=%FREEGLUT_INCLUDE_DIR% ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
    %PDAL_DIR%


rem    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
