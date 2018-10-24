pushd build

REM set CONDA_ROOT=C:\\Miniconda3-x64
SET PDAL_DRIVER_PATH="C:\projects\pdal\build\bin"
SET GEOTIFF_CSV=c:\\Minicoda3-x64\\Library\\share\\epsg_csv
cd "c:\projects\pdal\build"
ctest -V --output-on-failure
echo %CD%

popd
