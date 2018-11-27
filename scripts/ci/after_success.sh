#!/bin/bash

echo $TRAVIS_SECURE_ENV_VARS
if [ $TRAVIS_SECURE_ENV_VARS = "true" -a $TRAVIS_BRANCH = "1.8-maintenance ]
then
    echo "publishing website"
    ./build_docs.sh
    ./add_deploy_key.sh
    ./deploy_website.sh
    $TRAVIS_BUILD_DIR/doc/build /tmp
fi
