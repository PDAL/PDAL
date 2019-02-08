call "%CONDA_ROOT%\Scripts\activate.bat" base

pushd "scripts\\appveyor\\conda\\recipe"
dir
conda build .
popd
