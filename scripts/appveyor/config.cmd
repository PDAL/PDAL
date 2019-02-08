@echo off

set "CONDA_ROOT=C:\Miniconda3-x64"
set PATH=%CONDA_ROOT%;%CONDA_ROOT%\\scripts;%CONDA_ROOT%\\Library\\bin;%PATH%;C:\\Program Files (x86)\\CMake\\bin
conda config --set always_yes yes
conda config --add channels conda-forge
conda config --add channels anaconda
conda update -q conda
conda config --set auto_update_conda no
conda update -q --all
conda info
python -c "import sys; print(sys.version)"
python -c "import sys; print(sys.executable)"
python -c "import sys; print(sys.prefix)"
call "%CONDA_ROOT%\Scripts\activate.bat" base
conda install geotiff laszip nitro curl gdal pcl cmake eigen ninja libgdal geos zstd numpy xz libxml2 laz-perf qhull sqlite hdf5 oracle-instantclient numpy-base conda-build

