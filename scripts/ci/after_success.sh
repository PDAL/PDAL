#!/bin/bash

TRAVIS_BUILD_DIR=$1

echo "publishing website"

./build_docs.sh
./add_deploy_key.sh
./deploy_website.sh $TRAVIS_BUILD_DIR/doc/build /tmp
