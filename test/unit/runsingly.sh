#!/bin/bash

# run each test in the suite by itself -- for easier meleak testing

( for i in `ls -1 *.cpp */*.cpp */*/*.cpp` ; do echo `basename $i .cpp` ; done ) > ./suites.txt

wc -l ./suites.txt

 for i in `cat ./suites.txt`
 do
   [ "$i" == "Support" ] && continue
   [ "$i" == "TestConfig" ] && continue
   [ "$i" == "main" ] && continue
   
   echo "*** RUNNING TEST: $i ***"
   ../../bin/Debug/pdal_test -t $i
   
   if [ $? -ne 0 ] ; then echo "FAILED!" ; exit 1 ; fi
   
   echo ""
 done
 
 rm -f ./suites.txt
 