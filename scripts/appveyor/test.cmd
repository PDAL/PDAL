pushd build
set CURL_CA_INFO=C:\OSGeo4W64\bin\curl-ca-bundle.crt
set PROJSO=proj_5_2.dll
ctest -V --output-on-failure

popd
