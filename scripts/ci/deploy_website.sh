#!/bin/bash

builddir=$(pwd)/doc/build
destdir=$(pwd)/../pdal.github.io
branch="gh-action"
DATE=$(date +'%y.%m.%d %H:%M:%S')

#git clone git@github.com:PDAL/pdal.github.io.git $destdir
git clone https://${API_TOKEN_GITHUB}:x-oauth-basic@github.com
cd $destdir
git checkout -f -b $branch


cd $builddir/html
cp -rf * $destdir/

cd $builddir/latex/
cp PDAL.pdf $destdir/

cd $destdir
git config user.email "pdal@hobu.net"
git config user.name "PDAL Travis docsbot"

git add -A
git commit -m "update with results of commit https://github.com/PDAL/PDAL/commit/$GITHUB_SHA for ${DATE}"
git push origin $branch

