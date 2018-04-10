@echo off


REM if "%OSGEO4W_BUILD%" NEQ "ON" (
REM     REM only bother with this for OSGEO4W_BUILD situation
REM     echo "Not building OSGeo4W package"
REM     exit /b 0
REM )


call %OSGEO4W_ROOT%\bin\o4w_env.bat
call %OSGEO4W_ROOT%\bin\py3_env.bat
call "C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" amd64

set PATH=%PATH%;C:\Program Files (x86)\MSBuild\14.0\Bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\7-Zip;C:\Program Files\Microsoft Windows Performance Toolkit\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Tools\GitVersion;C:\Program Files (x86)\CMake\bin;C:\Program Files\Git\cmd;C:\Program Files\Git\usr\bin;C:\Program Files\AppVeyor\BuildAgent\

pushd c:\projects\pdal\build

nmake /f Makefile install  DESTDIR=C:\projects\pdal\install

popd

pushd c:\projects\pdal\install\osgeo4w64

tar jcvf ..\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 .
copy c:\pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2 c:\projects\pdal
echo "OSGeo4W64 build will be uploaded to https://s3.amazonaws.com/pdal/osgeo4w/pdal-%APPVEYOR_REPO_COMMIT%.tar.bz2"

popd
