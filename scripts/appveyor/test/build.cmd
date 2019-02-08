call "%CONDA_ROOT%\Scripts\activate.bat" base

pushd build
REM nmake /f Makefile
ninja
popd
