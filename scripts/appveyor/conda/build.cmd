call "%CONDA_ROOT%\Scripts\activate.bat" base

pushd "scripts\\appveyor\\conda"
dir
conda build .
popd
