#!/bin/bash

echo "building docs for $TRAVIS_BUILD_DIR/docs"
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc pdal/docs make doxygen
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc pdal/docs make html
docker run -v $TRAVIS_BUILD_DIR:/data -w /data/doc pdal/docs make latexpdf


