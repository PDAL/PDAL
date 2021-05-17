#!/bin/bash


if [ -z "${ANACONDA_TOKEN+x}" ]
then
    echo "Anaconda token is not set, not uploading"
    exit 0;
fi

ls
pwd
find .

if [ -z "${ANACONDA_TOKEN}" ]
then
    echo "Anaconda token is empty, not uploading"
    exit 0;
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

find . -name "*pdal*.bz2" -exec anaconda -t "$ANACONDA_TOKEN" upload --force --no-progress --user pdal-master  {} \;

