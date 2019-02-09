call "%CONDA_ROOT%\Scripts\activate.bat" base

pushd "scripts\\appveyor\\conda\\recipe"
REM pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor
dir
conda build  --output-folder c:\projects\pdal\build .
popd

pushd build
dir
popd
