pushd build

call "%CONDA_ROOT%\Scripts\activate.bat" base
REM set CONDA_ROOT=C:\\Miniconda3-x64
SET PDAL_DRIVER_PATH="C:\projects\pdal\build\bin"
SET GEOTIFF_CSV=c:\\Minicoda3-x64\\Library\\share\\epsg_csv
set PYTHONHOME=C:\\Miniconda3-x64\
set PYTHONPATH=C:\\Miniconda3-x64\\Lib
cd "c:\projects\pdal\build"
ctest -V --output-on-failure
echo %CD%

popd
