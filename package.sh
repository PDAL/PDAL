#!/bin/bash


# Build PDAL package, including RC number of specified
# ./package.sh
# ./package.sh RC1

RC=$1


version=`./bin/pdal-config --version`

package_name="PDAL-"$version"-src"



if [[ "$OSTYPE" == "linux-gnu" ]]; then
MD5="md5sum"
elif [[ "$OSTYPE" == "darwin"* ]]; then
        # Mac OSX
MD5="md5"
fi

make dist

extensions=".tar.gz .tar.bz2"
for ext in $extensions
do

    filename=$package_name$ext
    if [ -n "$RC" ]; then

        rcname="PDAL-"$version$RC$ext
        echo $rcname
        cp $filename $rcname
        `$MD5 $rcname > $rcname.md5`
    fi

    echo "$MD5 $filename > $filename.md5"
    `$MD5 $filename > $filename.md5`

done

# name=`echo $filename|cut -d'.' -f1-3`
# extension=`echo $filename|cut -d'.' -f4-`
# echo $name


# newname="$name$RC.$extension"
# mv $filename "$newname"
# `md5sum $newname > $newname.md5`
