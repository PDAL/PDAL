call "%CONDA_ROOT%\Scripts\activate.bat" base
call conda install conda-build
pushd "scripts\\appveyor\\conda\\recipe"
REM pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor
dir
call conda build  --output-folder c:\projects\pdal\build .
popd

pushd build
dir
popd
