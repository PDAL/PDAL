#!/usr/bin/env bash

PDAL_DEPEND_ON_NATIVE=false ./sbt "-212" "project core" publish
