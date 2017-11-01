@echo off

REM cmake --build . --target install --config Release
nmake /f Makefile install  DESTDIR=C:\projects\pdal\install

cd c:\projects\pdal\install\osgeo4w64

tar jcvf ..\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 .
copy c:\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 c:\projects\pdal
echo "OSGeo4W64 build will be uploaded to https://s3.amazonaws.com/pdal/osgeo4w/pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2"
