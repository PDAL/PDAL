call "%CONDA_ROOT%\Scripts\activate.bat" base

pushd "scripts\\appveyor\\conda\\recipe"
dir
conda build  --output c:\projects\pdal\build\pdal-%APPVEYOR_REPO_COMMIT%-py37_appveyor.tar.bz2 .
popd
