#!/bin/bash

if [ -n "$encrypted_6a5172b96922_key" ]; then
    openssl aes-256-cbc -K $encrypted_6a5172b96922_key -iv $encrypted_6a5172b96922_iv -in scripts/ci/pdaldocs-private.key.enc -out scripts/ci/pdaldocs-private.key -d
    cp scripts/ci/pdaldocs-private.key ~/.ssh/id_rsa
    chmod 600 ~/.ssh/id_rsa
    echo -e "Host *\n\tStrictHostKeyChecking no\n" > ~/.ssh/config
fi;


