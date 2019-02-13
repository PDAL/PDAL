call "%CONDA_ROOT%\Scripts\activate.bat" base
call conda install geotiff laszip nitro curl gdal pcl cmake eigen ninja libgdal geos zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 numpy-base tiledb conda-build -y
call conda config --set path_conflict clobber
pushd "scripts\\appveyor\\conda\\recipe"
REM pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor
mkdir build
dir
call conda build  --output-folder c:\projects\pdal\build .

dir
FOR /F %%I IN ('DIR c:\projects\pdal\build\win-64\*.bz2 /B /O:-D')  DO echo %%I > condaPackage
set /p CONDA_PACKAGE= < condaPackage
dir
DEL condaPackage
copy %CONDA_PACKAGE% c:\projects\pdal\pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor.tar.bz2

dir c:\projects\pdal
conda build purge
popd
