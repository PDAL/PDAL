mkdir build
cd build

cmake -G "Ninja" ^
      -DCMAKE_INSTALL_PREFIX:PATH="%LIBRARY_PREFIX%" ^
      -DCMAKE_BUILD_TYPE:STRING=Release ^
      -DCMAKE_LIBRARY_PATH="%LIBRARY_LIB%" ^
      -DCMAKE_INCLUDE_PATH="%INCLUDE_INC%" \^
      -DPython3_ROOT_DIR:FILEPATH="%LIBRARY_PREFIX%" ^
      -DBUILD_PLUGIN_GREYHOUND=ON ^
      -DBUILD_PLUGIN_PYTHON=ON ^
      -DBUILD_PLUGIN_PGPOINTCLOUD=ON ^
      -DBUILD_PLUGIN_SQLITE=ON ^
      -DBUILD_PLUGIN_I3S=ON ^
      -DBUILD_PLUGIN_ICEBRIDGE=ON ^
      -DBUILD_PLUGIN_HEXBIN=ON ^
      -DBUILD_PLUGIN_NITF=ON ^
      -DENABLE_CTEST=OFF ^
      -DWITH_TESTS=OFF ^
      -DWITH_ZLIB=ON ^
      -DWITH_LAZPERF=ON ^
      -DWITH_LASZIP=ON ^
      %SRC_DIR%
if errorlevel 1 exit 1

ninja
if errorlevel 1 exit 1

ninja install
if errorlevel 1 exit 1

set ACTIVATE_DIR=%PREFIX%\etc\conda\activate.d
set DEACTIVATE_DIR=%PREFIX%\etc\conda\deactivate.d
mkdir %ACTIVATE_DIR%
mkdir %DEACTIVATE_DIR%

copy %RECIPE_DIR%\scripts\activate.bat %ACTIVATE_DIR%\pdal-activate.bat
if errorlevel 1 exit 1

copy %RECIPE_DIR%\scripts\deactivate.bat %DEACTIVATE_DIR%\pdal-deactivate.bat
if errorlevel 1 exit 1

:: Copy unix shell activation scripts, needed by Windows Bash users
copy %RECIPE_DIR%\scripts\activate.sh %ACTIVATE_DIR%\pdal-activate.sh
if errorlevel 1 exit 1

copy %RECIPE_DIR%\scripts\deactivate.sh %DEACTIVATE_DIR%\pdal-deactivate.sh
if errorlevel 1 exit 1

