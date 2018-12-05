#!/bin/bash

TRAVIS_BUILD_DIR=$1
KEY=$2
KEY_INIT=$3

echo "publishing website"

echo "building docs"
./build_docs.sh $TRAVIS_BUILD_DIR

echo "adding key"
./add_deploy_key.sh pdaldocs-private.key $KEY $KEY_INIT

echo "deploying website"
./deploy_website.sh $TRAVIS_BUILD_DIR/doc/build /tmp
echo "done"
