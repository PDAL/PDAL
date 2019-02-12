call "%CONDA_ROOT%\Scripts\activate.bat" base
call conda install geotiff laszip nitro curl gdal pcl cmake eigen ninja libgdal geos zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 oracle-instantclient numpy-base tiledb conda-build
pushd "scripts\\appveyor\\conda\\recipe"
REM pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor
mkdir build
dir
conda build  --output-folder c:\projects\pdal\build .
popd

pushd build
dir
popd
