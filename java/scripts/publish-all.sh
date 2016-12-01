#!/usr/bin/env bash

./sbt "project core" publish && \
./sbt "-212" "project core" publish && \
./sbt "project core" publish-javastyle
