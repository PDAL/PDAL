pushd build

set CONDA_ROOT=C:\\Miniconda3-x64
SET PDAL_DRIVER_PATH="C:\projects\pdal\build\bin"
call %CONDA_ROOT%\\Scripts\\activate.bat base
cd "c:\projects\pdal\build"
ctest -V --output-on-failure
echo %CD%

popd
