#!/bin/bash

# Build PDAL package
# ./package.sh

if [ $# -gt 1 ]
then
    echo "Usage: package [RELEASE NAME]"
    exit
fi

GITSHA="$(git rev-parse HEAD)"

echo "Cutting release for SHA $GITSHA"

HERE=`pwd`
CONTAINER="pdal/alpinebase"
DOCKER="docker"

CONTAINERRUN="$DOCKER run -it -d --entrypoint /bin/sh -v $HERE:/data $CONTAINER"

CONTAINERID=`$CONTAINERRUN`
cat > docker-package.sh << "EOF"
#!/bin/sh

if [ $# -eq 1 ]
then
    RELNAME=$1
fi

git clone https://github.com/PDAL/PDAL.git;
cd /PDAL;
EOF


echo "git checkout $GITSHA" >> docker-package.sh


cat >> docker-package.sh << "EOF"
mkdir build; cd build;

if [ -n "${RELNAME+x}" ]
then
    cmake -G Ninja -DPDAL_VERSION_STRING=$RELNAME ..
else
    cmake -G Ninja ..
fi
ninja dist
chmod u+x ./bin/pdal-config
RELNAME=$(./bin/pdal-config --version)

OUTPUTDIR="/data/release-$RELNAME"
if [ ! -e $OUTPUTDIR ]
then
    mkdir $OUTPUTDIR
fi

extensions=".tar.gz .tar.bz2"
for ext in $extensions
do
    for filename in $(ls *$ext)
    do
        `md5sum $filename > $filename.md5`
        `sha256sum $filename > $filename.sha256sum`
        `sha512sum $filename > $filename.sha512sum`
        cp $filename $OUTPUTDIR
        cp $filename.md5 $OUTPUTDIR
        cp $filename.sha256sum $OUTPUTDIR
        cp $filename.sha512sum $OUTPUTDIR
    done
done

EOF

chmod +x docker-package.sh
docker cp docker-package.sh $CONTAINERID:/docker-package.sh

if [ $# -eq 1 ]
then
    RELNAME=$1
fi
docker exec -it $CONTAINERID /docker-package.sh $RELNAME

# run this to halt into the container
#docker exec -it $CONTAINERID bash

command="$DOCKER stop $CONTAINERID"
echo $command
$command

