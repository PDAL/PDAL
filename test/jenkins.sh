#!/usr/bin/env bash
# Run the tests in a clean PDAL repository
# "clean" == git clean -fdx

CMAKE_OPTS="-G \"Unix Makefiles\""


if [ "$1" == "PDAL-embed-boost" ]; then
    CMAKE_OPTS="$CMAKE_OPTS -C test/BuildSetup-embed.jenkins ."
else
    CMAKE_OPTS="$CMAKE_OPTS -C test/BuildSetup.jenkins ."
fi

#export ORACLE_HOME="/u01/app/oracle/product/11.2.0/dbhome_1"
export PATH="/var/lib/jenkins/bin:$PATH"
echo "running: cmake $CMAKE_OPTS"
eval cmake $CMAKE_OPTS
make
./bin/pdal_test test/data --catch_system_errors=no
