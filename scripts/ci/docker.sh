#!/bin/bash

# pdal/dock image has all of the Sphinx
# dependencies need to build PDAL's docs

docker pull pdal/dependencies
docker pull pdal/docs

