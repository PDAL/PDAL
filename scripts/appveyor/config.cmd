@echo off

set "CONDA_ROOT=C:\Miniconda3-x64"
set PATH=%CONDA_ROOT%;%CONDA_ROOT%\\scripts;%CONDA_ROOT%\\Library\\bin;%PATH%;C:\\Program Files (x86)\\CMake\\bin
conda config --set always_yes yes
conda config --add channels conda-forge
conda config --set auto_update_conda no
conda update -q --all
conda info
python -c "import sys; print(sys.version)"
python -c "import sys; print(sys.executable)"
python -c "import sys; print(sys.prefix)"

dir

