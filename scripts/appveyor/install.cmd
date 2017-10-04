@echo off

cmake --build . --target install --config Release
cd c:\pdalbin
dir 
tar jcvf ..\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 .
copy c:\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 c:\projects\pdal
echo "OSGeo4W64 build will be uploaded to https://s3.amazonaws.com/pdal/osgeo4w/pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2"
