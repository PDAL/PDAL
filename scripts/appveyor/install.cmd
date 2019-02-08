@echo off

mkdir c:\projects\pdal\conda
cd c:\projects\pdal\conda
git clone https://github.com/conda-forge/pdal-feedstock.git
cd pdal-feedstock

set "CONDA_ROOT=C:\Miniconda3-x64"
set PATH=%CONDA_ROOT%;%CONDA_ROOT%\\scripts;%CONDA_ROOT%\\Library\\bin;%PATH%;C:\\Program Files (x86)\\CMake\\bin

conda build .

REM pushd c:\projects\pdal\build

REM nmake /f Makefile install  DESTDIR=C:\projects\pdal\install
REM
REM set DESTDIR=C:\projects\pdal\install
REM ninja install

REM popd

REM pushd c:\projects\pdal\install\osgeo4w64

REM tar jcvf ..\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 .
REM copy c:\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 c:\projects\pdal
REM echo "OSGeo4W64 build will be uploaded to https://s3.amazonaws.com/pdal/osgeo4w/pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2"

REM popd
