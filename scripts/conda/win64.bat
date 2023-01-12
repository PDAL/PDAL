
mkdir build 
cd build 
echo "conda prefix" %CONDA_PREFIX%
REM set GENERATOR="Visual Studio 14 2015 Win64"
set GENERATOR="NMake Makefiles"
set GENERATOR="Ninja"

cmake -G %GENERATOR% ^
      -DCMAKE_BUILD_TYPE:STRING=RelWithDebInfo ^
      -DCMAKE_INSTALL_PREFIX:STRING="%CONDA_PREFIX%" ^
      -DCMAKE_LIBRARY_PATH:FILEPATH="=%CONDA_PREFIX%/Library/lib" ^
      -DBUILD_PLUGIN_I3S=ON ^
      -DBUILD_PLUGIN_E57=ON ^
	-DBUILD_PLUGIN_TILEDB=ON ^
      -DBUILD_PLUGIN_ICEBRIDGE=ON ^
      -DBUILD_PLUGIN_NITF=ON ^
      -DBUILD_PLUGIN_TILEDB=OFF ^
      -DWITH_TESTS=ON ^
      -DWITH_ZLIB=ON ^
      -DCMAKE_VERBOSE_MAKEFILE=OFF ^
      -DWITH_ZSTD=ON ^
      -DWITH_LZMA=ON ^
      -DLIBLZMA_LIBRARY:FILEPATH=%CONDA_PREFIX%\Library\lib\liblzma.lib ^
      -DZSTD_LIBRARY:FILEPATH=%CONDA_PREFIX%\Library\lib\libzstd.lib ^
      .. --debug-trycompile

call ninja
REM nmake /f Makefile
