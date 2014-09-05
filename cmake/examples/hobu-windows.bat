@echo off
 
:: This configure script is designed for the default Windows world, which means
:: you have OSGeo4W installed, including Oracle and GDAL and LASzip.

:: This configure script expects to be run from the PDAL root directory.

:: Pick your CMake GENERATOR.  (NMake will pick up architecture (x32, x64) from your environment)
set GENERATOR="NMake Makefiles"
rem set GENERATOR="Visual Studio 10 Win64"
rem set GENERATOR="Visual Studio 10"

:: Pick your build type
set BUILD_TYPE=Release
REM set BUILD_TYPE=Debug

:: Where is your PDAL build tree?
set PDAL_DIR=.

:: Where is your OSGeo4W installed (recommended basic way to satisfy dependent libs)
set OSGEO4W_DIR=C:\OSGeo4W64
set BOOST=c:\boost\1.55

:: Where is boost installed?
REM set PDAL_EMBED_BOOST=ON
rem set BOOST_DIR=c:\utils\boost_1_49_0
REM set BOOST_DIR=%PDAL_DIR%\boost

:: CARIS
REM set CARIS_ENABLED=ON
set CARIS_INCLUDE_DIR=%CARIS_DIR%\include
set CARIS_LIBRARY=%CARIS_DIR%\caris.lib

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
set LIBXML2_ENABLED=ON
set LIBXML2_INCLUDE_DIR=%OSGEO4W_DIR%\include
set LIBXML2_LIBRARIES=%OSGEO4W_DIR%\lib\libxml2.lib

:: IConv
set ICONV_ENABLED=ON
set ICONV_INCLUDE_DIR=%OSGEO4W_DIR%\include
set ICONV_LIBRARY=%OSGEO4W_DIR%\lib\iconv.lib

:: Python
set PYTHON_ENABLED=ON
REM set PYTHON_EXECUTABLE=%OSGEO4W_DIR\bin\python27.exe
REM set PYTHON_INCLUDE_DIR=%OSGEO4W_DIR\apps\python27\include
REM set PYTHON_LIBRARY=%OSGEO4W_DIR\apps\python27\libs\python27.lib



if EXIST CMakeCache.txt del CMakeCache.txt
del /S /Q CMakeFiles
cd %PDAL_DIR%/build
cmake -G %GENERATOR% ^
    -DBOOST_INCLUDEDIR=%BOOST% ^
    -DWITH_GDAL=%GDAL_ENABLED% ^
    -DCMAKE_INSTALL_PREFIX=c:\dev\PDAL\package ^
    -DWITH_GEOTIFF=%GEOTIFF_ENABLED% ^
    -DWITH_ORACLE=%ORACLE_ENABLED% ^
    -DWITH_PGPOINTCLOUD=ON ^
    -DWITH_LASZIP=%LASZIP_ENABLED% ^
    -DWITH_LIBXML2=%LIBXML2_ENABLED% ^
    -DWITH_ICONV=%ICONV_ENABLED% ^
	-DWITH_PYTHON=%PYTHON_ENABLED% ^
	-DWITH_NITRO=ON ^
	-DWITH_HEXER=ON ^
	-DPYTHON_EXECUTABLE=%OSGEO4W_DIR%\bin\python.exe ^
	-DPYTHON_INCLUDE_DIR=%OSGEO4W_DIR%\apps\python27\include ^
	-DPYTHON_LIBRARY=%OSGEO4W_DIR%\apps\python27\libs\python27.lib ^
	-DNUMPY_INCLUDE_DIR=%OSGEO4W_DIR%\apps\python27\lib\site-packages\numpy\core\include ^
	-DNUMPY_VERSION=1.5.1 ^
    -DCMAKE_BUILD_TYPE=%BUILD_TYPE% ^
    -DCMAKE_VERBOSE_MAKEFILE=OFF ^
	-DWITH_SQLITE=ON ^
    ..
    
rem    -DBOOST_INCLUDEDIR=%BOOST_DIR% ^
rem 	-DNUMPY_INCLUDE_DIR=%OSGEO4W_DIR%\apps\python27\lib\site-packages\numpy\core\include ^
rem 	-DPYTHONPATH=%OSGEO4W_DIR%\apps\python27\lib\site-packages ^
rem 	-DPYTHON_LIBRARY=%PYTHON_LIBRARY% ^
