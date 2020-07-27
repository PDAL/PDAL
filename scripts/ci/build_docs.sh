#!/bin/bash

TRAVIS_BUILD_DIR=$1
echo "building docs for $TRAVIS_BUILD_DIR/doc"
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc osgeo/proj-docs make doxygen
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc osgeo/proj-docs make html
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc osgeo/proj-docs make latexpdf
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc osgeo/proj-docs make spell


