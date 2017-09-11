#!/usr/bin/env bash

# --suffix: sets the suffix you want to publish lib with
# --signed: makes a PGP signed publish of the library, for maven central this sign is required

for i in "$@"
do
    case $i in
        --suffix=*)
            PDAL_VERSION_SUFFIX="${i#*=}"
            shift
            ;;
        --signed)
            SIGNED=true
            shift
            ;;
        *)
            ;;
    esac
done

export PDAL_VERSION_SUFFIX=${PDAL_VERSION_SUFFIX-"-SNAPSHOT"}
SIGNED=${SIGNED:-false}

COMMAND=publish

if ${SIGNED}; then
    COMMAND=publishSigned
fi

PDAL_DEPEND_ON_NATIVE=false ./sbt "-212" "project core" ${COMMAND}
PDAL_DEPEND_ON_NATIVE=false ./sbt "-212" "project core-scala" ${COMMAND}
