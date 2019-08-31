set CONDA_ENVIRO=pdal-build

call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64

call conda config --set always_yes yes
IF ERRORLEVEL 1 GOTO CLEANUP

call conda remove --name %CONDA_ENVIRO% -y --all
IF ERRORLEVEL 1 GOTO CLEANUP

call conda create --name %CONDA_ENVIRO% -y
IF ERRORLEVEL 1 GOTO CLEANUP

call conda activate %CONDA_ENVIRO%
IF ERRORLEVEL 1 GOTO CLEANUP

call conda install -y -c conda-forge ninja geotiff laszip nitro curl gdal cmake eigen ninja zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 tiledb 
IF ERRORLEVEL 1 GOTO CLEANUP

call conda install 	geotiff ^
					laszip ^
					nitro ^
					curl ^
					gdal ^
					cmake ^
					eigen ^
					ninja ^
					zstd ^
					numpy ^
					xz ^
					libxml2 ^
					laz-perf ^
					qhull ^
					sqlite ^
					hdf5 ^
					libxml2 ^
					tiledb

REM IF ERRORLEVEL 1 GOTO CLEANUP

:CLEANUP
call conda deactivate
popd
exit /b

