#!/usr/bin/env sh
# Run the tests in a clean PDAL repository
# "clean" == git clean -fdx

export ORACLE_HOME="/u01/app/oracle/product/11.2.0/dbhome_1"
export PATH="/var/lib/jenkins/local/bin:$PATH"
cmake -G "Unix Makefiles" -C "test/BuildSetup.jenkins" .
make
./bin/pdal_test test/data
