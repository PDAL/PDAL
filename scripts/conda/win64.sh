set CONDA_ENVIRO=pdal-build

del /s /q  %CONDA_ENVIRO%
mkdir %CONDA_ENVIRO%
pushd %CONDA_ENVIRO%

set GDAL_VERSION=2.2.4
set NUMPY_VERSION=1.15.3

call conda config --set always_yes yes
IF ERRORLEVEL 1 GOTO CLEANUP

call conda remove --name %CONDA_ENVIRO% -y --all
IF ERRORLEVEL 1 GOTO CLEANUP

call conda create --name %CONDA_ENVIRO% -y
IF ERRORLEVEL 1 GOTO CLEANUP

call %CONDA_PREFIX%\\Scripts\activate.bat %CONDA_ENVIRO%
IF ERRORLEVEL 1 GOTO CLEANUP

call conda config --add channels conda-forge
IF ERRORLEVEL 1 GOTO CLEANUP

call conda install geotiff laszip nitro curl gdal=%GDAL_VERSION% pcl cmake eigen ninja libgdal=%GDAL_VERSION% zstd numpy=%NUMPY_VERSION% xz libxml2 laz-perf qhull sqlite hdf5 oracle-instantclient numpy-base=%NUMPY_VERSION% tiledb

IF ERRORLEVEL 1 GOTO CLEANUP

REM set GENERATOR="Visual Studio 14 2015 Win64"
REM set GENERATOR="NMake Makefiles"
set GENERATOR="Ninja"

set ORACLE_HOME=%CONDA_PREFIX%
cmake -G %GENERATOR% ^
      -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo ^
      -DCMAKE_LIBRARY_PATH:FILEPATH="=%CONDA_PREFIX%/Library/lib" ^
      -DCMAKE_INCLUDE_PATH:FILEPATH="%CONDA_PREFIX%/Library/include" ^
      -DBUILD_PLUGIN_GREYHOUND=ON ^
      -DBUILD_PLUGIN_PCL=ON ^
      -DBUILD_PLUGIN_PYTHON=ON ^
      -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
      -DBUILD_PLUGIN_OCI=ON ^
      -DBUILD_PLUGIN_I3S=ON ^
      -DBUILD_PLUGIN_SQLITE=ON ^
      -DBUILD_PLUGIN_ICEBRIDGE=ON ^
      -DBUILD_PLUGIN_NITF=ON ^
      -DENABLE_CTEST=OFF ^
      -DWITH_TESTS=ON ^
      -DWITH_ZLIB=ON ^
      -DBUILD_PGPOINTCLOUD_TESTS=OFF ^
      -DBUILD_SQLITE_TESTS=OFF ^
      -DBUILD_OCI_TESTS=OFF ^
      -DCMAKE_VERBOSE_MAKEFILE=OFF ^
      -DWITH_LAZPERF=ON ^
      -DWITH_LASZIP=ON ^
      .. --debug-trycompile

call ninja
IF ERRORLEVEL 1 GOTO CLEANUP

:CLEANUP
call conda deactivate
popd
exit /b

