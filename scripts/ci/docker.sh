#!/bin/bash

# pdal/doc image has all of the Sphinx
# dependencies need to build Entwine's docs

docker pull pdal/dependencies
docker pull pdal/docs

