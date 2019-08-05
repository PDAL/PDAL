call "%CONDA_ROOT%\Scripts\activate.bat" base
call conda install geotiff laszip nitro curl gdal cmake eigen ninja libgdal zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 tiledb conda-build ninja -y
call conda config --set path_conflict clobber
pushd "scripts\\appveyor\\conda\\recipe"
REM pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor
mkdir build
call conda build  --output-folder c:\projects\pdal\build .

FOR /F %%I IN ('DIR c:\projects\pdal\build\win-64\*.bz2 /B /O:-D')  DO echo %%I > condaPackage
set /p CONDA_PACKAGE= < condaPackage
DEL condaPackage
copy c:\projects\pdal\build\win-64\%CONDA_PACKAGE% c:\projects\pdal\pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor.tar.bz2

popd
