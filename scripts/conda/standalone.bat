
mkdir build
cd build
echo "conda prefix" %CONDA_PREFIX%


cmake .. ^
      -DSTANDALONE=ON ^
      -DCMAKE_INSTALL_PREFIX="%CONDA_PREFIX%" ^
      -DCMAKE_BUILD_TYPE=Release ^
      -DPDAL_DIR="%CONDA_PREFIX%" ^
      -DBUILD_PLUGIN_DRACO=ON ^
  -G Ninja
REM ninja
REM ninja install


