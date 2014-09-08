#!/bin/bash
if [[ "$TRAVIS" != "true" ]] ; then
	echo "Running this script makes no sense outside of travis-ci.org"
	exit 1
fi

# Functions
tmstamp() { echo -n "[$(date '+%H:%M:%S')]" ; }

# Environment
NUMTHREADS=4
if [[ -f /sys/devices/system/cpu/online ]]; then
	# Calculates 1.5 times physical threads
	NUMTHREADS=$(( ( $(cut -f 2 -d '-' /sys/devices/system/cpu/online) + 1 ) * 15 / 10  ))
fi
#NUMTHREADS=1 # disable MP
export NUMTHREADS

# pdal_test segfaults when built against external g++-built boost,
# and I haven't found a good boost package built with clang yet
if [[ "$CXX" == "clang++" ]]
then
    export PDAL_CMAKE_GENERATOR="Ninja"
else
    export PDAL_CMAKE_GENERATOR="Unix Makefiles"
    export PDAL_EMBED_BOOST="OFF"
fi
