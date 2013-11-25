#!/bin/bash
shopt -s expand_aliases
alias unzip-stream="python -c \"import zipfile,sys,StringIO;zipfile.ZipFile(StringIO.StringIO(sys.stdin.read())).extractall(sys.argv[1] if len(sys.argv) == 2 else '.')\""

GTXS=("http://download.osgeo.org/proj/vdatum/vertcon/vertconc.gtx" 
      "http://download.osgeo.org/proj/vdatum/vertcon/vertcone.gtx" 
      "http://download.osgeo.org/proj/vdatum/vertcon/vertconw.gtx" 
      "http://download.osgeo.org/proj/vdatum/egm96_15/egm96_15.gtx" 
      "http://download.osgeo.org/proj/vdatum/egm08_25/egm08_25.gtx" 
      )

ZIPS=("http://download.osgeo.org/proj/vdatum/usa_geoid2012.zip"
      "http://download.osgeo.org/proj/vdatum/usa_geoid2009.zip"
      "http://download.osgeo.org/proj/vdatum/usa_geoid2003.zip"
      "http://download.osgeo.org/proj/vdatum/usa_geoid1999.zip"
      )

extract_path="/home/vagrant/datum"
export_path="/usr/share/proj/"
for ZIP in "${ZIPS[@]}"
do
    :
    filename=$(basename "$ZIP")
    extension="${filename##*.}"
    filename="${filename%.*}"
    wget $ZIP -O - | unzip-stream $extract_path
    mv $extract_path/$filename/* $export_path
    rm -rf $extract_path/$filename
done


cd $export_path
for GTX in "${GTXS[@]}"
do
    :
    wget $GTX
done

chmod -R 775 $export_path