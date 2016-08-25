#!/bin/bash

BUILDDIR=/data
cd /home/pdal/pdal
git pull
cd doc
make doxygen BUILDDIR=$BUILDDIR
make html BUILDDIR=$BUILDDIR
