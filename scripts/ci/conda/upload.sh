#!/bin/bash


if [ -z "${ANACONDA_TOKEN+x}" ]
then
    echo "Anaconda token is not set, not uploading"
    exit 1;
fi

ls
pwd
find .

set PACKAGE_PATTERN="*pdal*.conda"
if [[ -n `find . -name "*pdal*.conda"` ]]; then
    echo "Found packages to upload"
else
    echo "No packages matching $PACKAGE_PATTERN to upload were found"
    exit 1;
fi

export CI_PLAT=""
if grep -q "windows" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="win"
fi

if grep -q "ubuntu" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="linux"
fi

if grep -q "macos" <<< "$PDAL_PLATFORM"; then
    CI_PLAT="osx"
fi



echo "Anaconda token is available, attempting to upload"

conda install -c conda-forge anaconda-client -y

find . -name $PACKAGE_PATTERN -exec anaconda -t "$ANACONDA_TOKEN" upload --force --no-progress --user pdal-master  {} \;

