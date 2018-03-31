REM set PATH=%OSGEO4W_ROOT%\bin;C:\Program Files (x86)\MSBuild\14.0\Bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\7-Zip;C:\Program Files\Microsoft Windows Performance Toolkit\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Tools\GitVersion;C:\Program Files (x86)\CMake\bin;C:\Program Files\Git\cmd;C:\Program Files\Git\usr\bin;C:\Program Files\AppVeyor\BuildAgent\
REM call %OSGEO4W_ROOT%\bin\o4w_env.bat
REM call %OSGEO4W_ROOT%\bin\py3_env.bat
REM set PATH=C:\Program Files (x86)\MSBuild\14.0\Bin;%PATH%
REM set PATH=%PATH%;C:\Program Files (x86)\MSBuild\14.0\Bin;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\7-Zip;C:\Program Files\Microsoft Windows Performance Toolkit\;C:\Program Files (x86)\Windows Kits\8.1\Windows Performance Toolkit\;C:\Tools\GitVersion;C:\Program Files (x86)\CMake\bin;C:\Program Files\Git\cmd;C:\Program Files\Git\usr\bin;C:\Program Files\AppVeyor\BuildAgent\

pushd build

ctest -V --output-on-failure

popd
