#!/bin/bash

KEY_FILE=$1
KEY=$2
INIT_VECTOR=$3

openssl aes-256-cbc -K $KEY -iv $INIT_VECTOR -in $KEY_FILE.enc \
    -out $KEY_FILE -d
cp $KEY_FILE ~/.ssh/id_rsa
chmod 600 ~/.ssh/id_rsa
echo -e "Host *\n\tStrictHostKeyChecking no\n" > ~/.ssh/config

