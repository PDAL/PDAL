pushd build

set CONDA_ROOT="C:\Miniconda3-x64"
SET PDAL_PLUGIN_INSTALL_PATH="C:/projects/pdal/build/bin"
set PATH="%CONDA_ROOT%;%CONDA_ROOT%\scripts;%CONDA_ROOT%\Library\bin;%PATH%;C:\Program Files (x86)\CMake\bin"
call "%CONDA_ROOT%\Scripts\activate.bat" base
ctest -V --output-on-failure

popd
