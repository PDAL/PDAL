pushd build

echo %CD%
set CONDA_ROOT="C:\Miniconda3-x64"
SET PDAL_PLUGIN_INSTALL_PATH="C:/projects/pdal/build/bin"
call "%CONDA_ROOT%\Scripts\activate.bat" base
set PATH="%PATH%;C:\Program Files (x86)\CMake\bin"
echo %CD%
cd "c:\projects\pdal\build"
echo %CD%
ctest -V --output-on-failure
echo %CD%

popd
